#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "kalman.h"
#include "motor.h"
#include "pid.h"

// 1 = flight controller, 0 = motor ramp test only.
#define ENABLE_MPU6050 1

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define SAMPLE_PERIOD_US 10000          // 100 Hz
#define BIAS_CAL_SAMPLES 1000           // ~1 s of stationary samples

#if ENABLE_MPU6050
// Motor index == GPIO. Axes are the ones printed on the GY-521:
//   +roll  (right-hand about +X) lifts the +Y side (GP1).
//   +pitch (right-hand about +Y) drops the +X side (GP3).
#define MOTOR_NEG_X 0
#define MOTOR_POS_Y 1
#define MOTOR_NEG_Y 2
#define MOTOR_POS_X 3

// +1 if the GP0/GP3 props spin clockwise seen from above, -1 if GP1/GP2 do.
#define YAW_SIGN 1.0f

#define HOVER_THROTTLE_PERCENT  30.0f   // tune until the drone just holds altitude
#define IDLE_THROTTLE_PERCENT    5.0f   // lowest command while armed so motors never stall
#define SPOOLUP_TIME_S           3.0f
#define FLIGHT_TIME_S           10.0f   // no RC link, so the flight ends on a timer
#define SPOOLDOWN_TIME_S         3.0f
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

static void failsafe(const char *reason) {
    motor_set_throttle_percent(0.0f);
    printf("FAILSAFE: %s - motors stopped\n", reason);
    while (true) sleep_ms(1000);
}

static float clamp_throttle(float t) {
    if (t < IDLE_THROTTLE_PERCENT) return IDLE_THROTTLE_PERCENT;
    if (t > 100.0f) return 100.0f;
    return t;
}

// + configuration mixer. Negative u_roll/u_pitch means "tilted positive, push back".
static void mix_and_output(float throttle, float u_roll, float u_pitch, float u_yaw) {
    float yaw = YAW_SIGN * u_yaw;
    motor_set_motor_throttle_percent(MOTOR_POS_Y, clamp_throttle(throttle + u_roll  - yaw));
    motor_set_motor_throttle_percent(MOTOR_NEG_Y, clamp_throttle(throttle - u_roll  - yaw));
    motor_set_motor_throttle_percent(MOTOR_NEG_X, clamp_throttle(throttle + u_pitch + yaw));
    motor_set_motor_throttle_percent(MOTOR_POS_X, clamp_throttle(throttle - u_pitch + yaw));
}

// Throttle profile: spool up -> hold hover -> spool down. Returns < 0 when done.
static float throttle_schedule(float t) {
    if (t < SPOOLUP_TIME_S) return HOVER_THROTTLE_PERCENT * t / SPOOLUP_TIME_S;
    if (t < FLIGHT_TIME_S)  return HOVER_THROTTLE_PERCENT;
    float down = t - FLIGHT_TIME_S;
    if (down < SPOOLDOWN_TIME_S)
        return HOVER_THROTTLE_PERCENT * (1.0f - down / SPOOLDOWN_TIME_S);
    return -1.0f;
}

// Convert raw accelerometer vector to roll/pitch in degrees.
// At rest, gravity points along -Z when the board is flat.
static void accel_to_angles(const vector_3d *a, float accel_scale,
                            float *roll, float *pitch) {
    float ax = a->x / accel_scale;
    float ay = a->y / accel_scale;
    float az = a->z / accel_scale;

    *roll  = atan2f(ay, az) * RAD_TO_DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// Average the gyro while the device is stationary to seed the bias.
// Returns bias in °/s on each axis.
static void calibrate_gyro_bias(float gyro_scale,
                                float *bias_x, float *bias_y, float *bias_z) {
    double sx = 0, sy = 0, sz = 0;
    int n = 0;

    for (int i = 0; i < BIAS_CAL_SAMPLES; i++) {
        vector_3d g;
        if (mpu6050_read_gyro(&g)) {   // see note below about signature
            sx += g.x;
            sy += g.y;
            sz += g.z;
            n++;
        }
        sleep_us(SAMPLE_PERIOD_US);
    }

    if (n == 0) {
        *bias_x = *bias_y = *bias_z = 0.0f;
        return;
    }
    *bias_x = (float)(sx / n) / gyro_scale;
    *bias_y = (float)(sy / n) / gyro_scale;
    *bias_z = (float)(sz / n) / gyro_scale;
}

#endif  // ENABLE_MPU6050

int main(void) {
    stdio_init_all();
    
    // Let the USB CDC serial port enumerate before we print.
    sleep_ms(5000);

    // Start sending the idle pulse immediately so the ESC can initialize.
    motor_init();

#if ENABLE_MPU6050
    if (!mpu6050_init()) {
        printf("MPU6050 init failed — halting\n");
        while (true) sleep_ms(1000);
    }

    const float accel_scale = mpu6050_accel_scale();   // 16384.0f
    const float gyro_scale  = mpu6050_gyro_scale();    // 131.0f

    printf("MPU6050 Kalman filter — accel_scale=%.1f gyro_scale=%.3f\n",
           accel_scale, gyro_scale);
    printf("Keep the board still during bias calibration...\n");

    float bias_x, bias_y, bias_z;
    calibrate_gyro_bias(gyro_scale, &bias_x, &bias_y, &bias_z);
    printf("Gyro bias: x=%.3f y=%.3f z=%.3f °/s\n", bias_x, bias_y, bias_z);

    kalman_t roll_kf, pitch_kf;

    // Seed filters with a first accel reading so they start near reality.
    vector_3d a0, g0;
    if (mpu6050_read_sample(&a0, &g0)) {
        float r0, p0;
        accel_to_angles(&a0, accel_scale, &r0, &p0);
        kalman_init(&roll_kf,  r0);
        kalman_init(&pitch_kf, p0);
        // Seed bias estimates directly so the filter doesn't spend
        // hundreds of milliseconds learning what we already know.
        roll_kf.bias  = bias_x;
        pitch_kf.bias = bias_y;
    } else {
        kalman_init(&roll_kf,  0.0f);
        kalman_init(&pitch_kf, 0.0f);
    }
#endif  // ENABLE_MPU6050

    printf("Arming ESC...\n");
    motor_arm();

#if !ENABLE_MPU6050
    // Ramp GP1-3 from 10% to 20% in 5% steps (1 s each, 3 s at 20%), then back to 0%.
    for (int percent = 10; percent <= 20; percent += 5) {
        for (unsigned m = 1; m < MOTOR_COUNT; m++) {
            motor_set_motor_throttle_percent(m, (float)percent);
        }
        printf("Motors 1-3 throttle %d%%\n", percent);
        sleep_ms(percent == 20 ? 3000 : 1000);
    }
    motor_set_throttle_percent(0.0f);
    printf("Motors 1-3 throttle 0%%\n");
    while (true) sleep_ms(1000);
#else
    uint64_t last_us = time_us_64();
    const uint64_t start_us = last_us;
    absolute_time_t next_tick = get_absolute_time();
    int read_failures = 0;
    unsigned loop_count = 0;

    while (true) {
        next_tick = delayed_by_us(next_tick, SAMPLE_PERIOD_US);
        sleep_until(next_tick);

        vector_3d accel, gyro;
        if (!mpu6050_read_sample(&accel, &gyro)) {
            // I2C hiccup — skip this cycle rather than feed zeros
            // into the filter, which would yank the angle.
            if (++read_failures >= MAX_READ_FAILURES) failsafe("IMU read failures");
            continue;
        }
        read_failures = 0;

        uint64_t now_us = time_us_64();
        float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        // Guard against pathological dt (first iteration, USB hiccup, etc.)
        if (dt <= 0.0f || dt > 0.1f) {
            dt = SAMPLE_PERIOD_US * 1e-6f;
        }

        float roll_acc, pitch_acc;
        accel_to_angles(&accel, accel_scale, &roll_acc, &pitch_acc);

        float gyro_roll  = gyro.x / gyro_scale;   // °/s
        float gyro_pitch = gyro.y / gyro_scale;

        float roll  = kalman_update(&roll_kf,  roll_acc,
                                    gyro_roll,  dt);
        float pitch = kalman_update(&pitch_kf, pitch_acc,
                                    gyro_pitch, dt);

        if (fabsf(roll) > MAX_TILT_DEG || fabsf(pitch) > MAX_TILT_DEG) {
            failsafe("tilt limit exceeded");
        }

        float throttle = throttle_schedule((now_us - start_us) * 1e-6f);
        if (throttle < 0.0f) failsafe("flight timer finished");

        float roll_rate  = gyro_roll  - roll_kf.bias;
        float pitch_rate = gyro_pitch - pitch_kf.bias;
        float yaw_rate   = gyro.z / gyro_scale - bias_z;

        bool integrate = throttle >= INTEGRATE_ABOVE_PERCENT;
        float u_roll  = pid_update(&roll_pid,  0.0f, roll,     roll_rate,  dt, integrate);
        float u_pitch = pid_update(&pitch_pid, 0.0f, pitch,    pitch_rate, dt, integrate);
        float u_yaw   = pid_update(&yaw_pid,   0.0f, yaw_rate, 0.0f,       dt, integrate);

        mix_and_output(throttle, u_roll, u_pitch, u_yaw);

        if (++loop_count % PRINT_EVERY_N_LOOPS == 0) {
            printf("thr=%5.1f roll=%6.2f pitch=%6.2f yaw_rate=%6.1f | u_r=%5.2f u_p=%5.2f u_y=%5.2f\n",
                   throttle, roll, pitch, yaw_rate, u_roll, u_pitch, u_yaw);
        }
    }
#endif  // ENABLE_MPU6050
}