#include "flight_controller.h"

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "attitude.h"
#include "failsafe.h"
#include "flight_log.h"
#include "flight_plan.h"
#include "mixer.h"
#include "pid.h"

#define LOOP_PERIOD_US          ATTITUDE_SAMPLE_PERIOD_US
#define INTEGRATE_ABOVE_PERCENT 10.0f   // freeze I-terms on the ground to avoid windup
#define MAX_TILT_DEG            45.0f
#define MAX_READ_FAILURES       50      // consecutive IMU failures (~100 ms)
#define PRINT_EVERY_N_LOOPS     50      // 10 Hz
#define GRAVITY_M_S2            9.81f
#define ACCEL_UP_LIMIT_G        0.5f    // leg chatter and impacts clip the accel at +2 g
#define TOUCHDOWN_LAG_M         0.08f   // in-air peak was 5 cm (log 16)
#define DESCENT_REF_TAU_S       0.25f   // vz can still be +0.2 when descent starts (log 21)
#define LANDED_THROTTLE_PERCENT 10.0f   // spool-up start: spins without lifting
#define LANDED_RAMP_PERCENT_S   50.0f   // hover -> idle in ~0.3 s
#define STILL_RATE_DPS          20.0f
#define STILL_TIME_S            1.0f
#define LANDED_TIMEOUT_S        3.0f

// Disturbance observer, in x = (m_a^2 - m_b^2)/80 units. Plant from the log 19 pitch probe:
// angular accel = DOB_B * (lag(command ~16 ms ago) + d), lag ~40 ms.
#define DOB_B                   550.0f  // deg/s^2 per x
#define DOB_DELAY_LOOPS         8
#define DOB_LAG_S               0.04f
#define DOB_T_S                 0.15f   // estimate filter; sim stable over b 340-850, delay+lag 40-100 ms
#define DOB_LIMIT_X             1.0f    // log 20 needed +0.9 roll / -0.8 pitch; sim stable to 2
#define DOB_MIN_HEIGHT_M        0.03f   // legs on the ground push back

// Outputs are in throttle percent. Angle loops: P in %/deg, D in %/(deg/s).
// ~70 ms motor delay: .06/.12/.010 oscillated (log 12); .025/.04/.014 drifted sideways (log 13).
static pid_ctrl_t roll_pid  = { .kp = 0.04f, .ki = 0.08f, .kd = 0.014f,
                                .i_limit = 8.0f, .out_limit = 15.0f };
static pid_ctrl_t pitch_pid = { .kp = 0.04f, .ki = 0.08f, .kd = 0.014f,
                                .i_limit = 8.0f, .out_limit = 15.0f };
// Yaw holds rate = 0 so motor torque imbalance doesn't spin the frame.
// Logs need a steady ~+1.7% yaw output; ki 0.5 builds it in ~0.3 s.
static pid_ctrl_t yaw_pid   = { .kp = 0.10f, .ki = 0.50f, .kd = 0.0f,
                                .i_limit = 5.0f, .out_limit = 10.0f };
// Climb rate: P in %/(m/s), I in %/m. ~0.7 m/s^2 per % near hover => ~1 Hz crossover.
static pid_ctrl_t vz_pid    = { .kp = 8.0f, .ki = 8.0f, .kd = 0.0f,
                                .i_limit = 5.0f, .out_limit = 8.0f };

typedef struct {
    float hist[DOB_DELAY_LOOPS];   // past commands, x units
    unsigned head;
    float x_pred;                  // command as the props are delivering it now
    float w_f, x_f;                // both sides low-passed by DOB_T_S
    float d_hat;
} dob_t;

static dob_t roll_dob, pitch_dob;

static void dob_reset(dob_t *o, float rate) {
    o->w_f = rate;
    o->x_f = 0.0f;
    o->d_hat = 0.0f;
}

// d = accel / b - x_pred, with accel taken as (rate - LPF(rate)) / T so nothing is differentiated.
static void dob_update(dob_t *o, float rate, float dt) {
    o->w_f += (rate - o->w_f) * dt / DOB_T_S;
    o->x_f += (o->x_pred - o->x_f) * dt / DOB_T_S;
    float d = (rate - o->w_f) / (DOB_B * DOB_T_S) - o->x_f;
    o->d_hat = fmaxf(-DOB_LIMIT_X, fminf(d, DOB_LIMIT_X));
}

static void dob_push(dob_t *o, float x, float dt) {
    float delayed = o->hist[o->head];
    o->hist[o->head] = x;
    o->head = (o->head + 1) % DOB_DELAY_LOOPS;
    o->x_pred += (delayed - o->x_pred) * dt / DOB_LAG_S;
}

bool flight_controller_init(void) {
    return attitude_init();
}

void flight_controller_run(void) {
    uint64_t last_us = time_us_64();
    const uint64_t start_us = last_us;
    absolute_time_t next_tick = get_absolute_time();
    int read_failures = 0;
    unsigned loop_count = 0;
    float vz = 0.0f;   // m/s, up positive; integrated accel, held at 0 on the ground
    float z = 0.0f;    // m above the liftoff point, integrated vz
    float descent_lag = 0.0f;   // m the estimate trails the commanded descent
    float vz_ref = 0.0f;        // m/s the vz loop could reach by now
    bool dob_active = false;    // airborne at least once; afterwards held near the ground
    bool landed = false;        // touchdown or end of plan: idle until still, then stop
    float landed_s = 0.0f, still_s = 0.0f, last_throttle = 0.0f;

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
        // Under thrust the accel can't see tilt and vibration biases it (~+8 deg pitch, logs 13-14).
        if (!attitude_update(dt, false, &att)) {
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

        flight_setpoint_t sp = { 0 };
        if (!landed && !flight_plan_setpoint((now_us - start_us) * 1e-6f, &sp)) landed = true;

        float throttle;
        if (landed) {
            throttle = fmaxf(LANDED_THROTTLE_PERCENT, last_throttle - LANDED_RAMP_PERCENT_S * dt);
            bool still = fabsf(att.roll_rate)  < STILL_RATE_DPS &&
                         fabsf(att.pitch_rate) < STILL_RATE_DPS &&
                         fabsf(att.yaw_rate)   < STILL_RATE_DPS;
            still_s = still ? still_s + dt : 0.0f;
            landed_s += dt;
            if (still_s >= STILL_TIME_S) failsafe("landed");
            if (landed_s >= LANDED_TIMEOUT_S) failsafe("landing timeout");
        } else if (sp.velocity_loop) {
            throttle = sp.throttle;
            vz += fmaxf(-ACCEL_UP_LIMIT_G, fminf(att.accel_up, ACCEL_UP_LIMIT_G)) * GRAVITY_M_S2 * dt;
            throttle += pid_update(&vz_pid, sp.climb_rate, vz, 0.0f, dt, true);
            z += vz * dt;
            if (sp.climb_rate < 0.0f) {
                vz_ref += (sp.climb_rate - vz_ref) * dt / DESCENT_REF_TAU_S;
                descent_lag += (vz - vz_ref) * dt;
                // Near-hover throttle on the ground lets it skate (logs 15-16). z can be ~10 cm off
                // (log 16), but on the ground vz stops following the setpoint, so the lag grows.
                if (z <= 0.0f || descent_lag >= TOUCHDOWN_LAG_M) landed = true;
            } else {
                vz_ref = vz;
            }
        } else {
            throttle = sp.throttle;
            vz = 0.0f;
            z = 0.0f;
        }
        last_throttle = throttle;

        bool integrate = !landed && throttle >= INTEGRATE_ABOVE_PERCENT;
        float u_roll  = pid_update(&roll_pid,  0.0f, att.roll,     att.roll_rate,  dt, integrate);
        float u_pitch = pid_update(&pitch_pid, 0.0f, att.pitch,    att.pitch_rate, dt, integrate)
                        + sp.pitch_probe;
        float u_yaw   = pid_update(&yaw_pid,   0.0f, att.yaw_rate, 0.0f,           dt, integrate);

        float kx = throttle / 20.0f;   // x per % of u
        if (landed) {
            dob_reset(&roll_dob,  att.roll_rate);
            dob_reset(&pitch_dob, att.pitch_rate);
        } else if (sp.velocity_loop && z > DOB_MIN_HEIGHT_M) {
            dob_active = true;
            dob_update(&roll_dob,  att.roll_rate,  dt);
            dob_update(&pitch_dob, att.pitch_rate, dt);
        } else if (!dob_active) {
            dob_reset(&roll_dob,  att.roll_rate);
            dob_reset(&pitch_dob, att.pitch_rate);
        }
        u_roll  -= roll_dob.d_hat  / kx;
        u_pitch -= pitch_dob.d_hat / kx;
        dob_push(&roll_dob,  kx * u_roll,  dt);
        dob_push(&pitch_dob, kx * u_pitch, dt);

        mixer_output(throttle, u_roll, u_pitch, u_yaw);
        flight_log_record(throttle, &att, u_roll, u_pitch, u_yaw, vz);

        if (++loop_count % PRINT_EVERY_N_LOOPS == 0) {
            printf("thr=%5.1f roll=%6.2f pitch=%6.2f yaw_rate=%6.1f vz=%5.2f | u_r=%5.2f u_p=%5.2f u_y=%5.2f\n",
                   throttle, att.roll, att.pitch, att.yaw_rate, vz, u_roll, u_pitch, u_yaw);
        }
    }
}
