#include "flight_controller.h"

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "attitude.h"
#include "failsafe.h"
#include "flight_plan.h"
#include "mixer.h"
#include "pid.h"

#define LOOP_PERIOD_US          ATTITUDE_SAMPLE_PERIOD_US
#define INTEGRATE_ABOVE_PERCENT 20.0f   // freeze I-terms on the ground to avoid windup
#define MAX_TILT_DEG            45.0f
#define MAX_READ_FAILURES       10      // consecutive IMU failures (~100 ms)
#define PRINT_EVERY_N_LOOPS     10

// Outputs are in throttle percent. Angle loops: P in %/deg, D in %/(deg/s).
static pid_ctrl_t roll_pid  = { .kp = 0.15f, .ki = 0.05f, .kd = 0.04f,
                                .i_limit = 5.0f, .out_limit = 15.0f };
static pid_ctrl_t pitch_pid = { .kp = 0.15f, .ki = 0.05f, .kd = 0.04f,
                                .i_limit = 5.0f, .out_limit = 15.0f };
// Yaw holds rate = 0 so motor torque imbalance doesn't spin the frame.
static pid_ctrl_t yaw_pid   = { .kp = 0.10f, .ki = 0.05f, .kd = 0.0f,
                                .i_limit = 5.0f, .out_limit = 10.0f };

bool flight_controller_init(void) {
    return attitude_init();
}

void flight_controller_run(void) {
    uint64_t last_us = time_us_64();
    const uint64_t start_us = last_us;
    absolute_time_t next_tick = get_absolute_time();
    int read_failures = 0;
    unsigned loop_count = 0;

    while (true) {
        next_tick = delayed_by_us(next_tick, LOOP_PERIOD_US);
        sleep_until(next_tick);

        uint64_t now_us = time_us_64();
        float dt = (now_us - last_us) * 1e-6f;

        // Guard against pathological dt (first iteration, USB hiccup, etc.)
        if (dt <= 0.0f || dt > 0.1f) {
            dt = LOOP_PERIOD_US * 1e-6f;
        }

        attitude_t att;
        if (!attitude_update(dt, &att)) {
            // I2C hiccup — skip this cycle rather than feed zeros
            // into the filter, which would yank the angle.
            if (++read_failures >= MAX_READ_FAILURES) failsafe("IMU read failures");
            continue;
        }
        read_failures = 0;
        last_us = now_us;

        if (fabsf(att.roll) > MAX_TILT_DEG || fabsf(att.pitch) > MAX_TILT_DEG) {
            failsafe("tilt limit exceeded");
        }

        float throttle = flight_plan_throttle((now_us - start_us) * 1e-6f);
        if (throttle < 0.0f) failsafe("flight timer finished");

        bool integrate = throttle >= INTEGRATE_ABOVE_PERCENT;
        float u_roll  = pid_update(&roll_pid,  0.0f, att.roll,     att.roll_rate,  dt, integrate);
        float u_pitch = pid_update(&pitch_pid, 0.0f, att.pitch,    att.pitch_rate, dt, integrate);
        float u_yaw   = pid_update(&yaw_pid,   0.0f, att.yaw_rate, 0.0f,           dt, integrate);

        mixer_output(throttle, u_roll, u_pitch, u_yaw);

        if (++loop_count % PRINT_EVERY_N_LOOPS == 0) {
            printf("thr=%5.1f roll=%6.2f pitch=%6.2f yaw_rate=%6.1f | u_r=%5.2f u_p=%5.2f u_y=%5.2f\n",
                   throttle, att.roll, att.pitch, att.yaw_rate, u_roll, u_pitch, u_yaw);
        }
    }
}
