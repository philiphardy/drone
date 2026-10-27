#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "kalman.h"
#include "motor.h"

// Set to 1 to re-enable the MPU6050 / Kalman filter code.
#define ENABLE_MPU6050 0

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define SAMPLE_PERIOD_US 10000          // 100 Hz
#define BIAS_CAL_SAMPLES 1000           // ~1 s of stationary samples

#if ENABLE_MPU6050
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

    // Arm (zero throttle for >= 3 s), ramp up from 0% to 2% in 0.1% steps,
    // hold at 2% for 3 s, then drop back to 0%.
    // Loop in tenths of a percent to avoid accumulating float error.
    printf("Arming ESC...\n");
    motor_arm();
    for (int tenths = 0; tenths <= 20; tenths++) {
        motor_set_throttle_percent(tenths / 10.0f);
        printf("Throttle %d.%d%%\n", tenths / 10, tenths % 10);
        sleep_ms(tenths == 20 ? 3000 : 1000);
    }
    motor_set_throttle_percent(0.0f);
    printf("Throttle 0%%\n");

#if !ENABLE_MPU6050
    while (true) sleep_ms(1000);
#else
    uint64_t last_us = time_us_64();

    while (true) {
        vector_3d accel, gyro;
        if (!mpu6050_read_sample(&accel, &gyro)) {
            // I2C hiccup — skip this cycle rather than feed zeros
            // into the filter, which would yank the angle.
            sleep_us(SAMPLE_PERIOD_US);
            continue;
        }

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

        printf("roll=%7.2f  pitch=%7.2f  (acc r=%7.2f p=%7.2f)\n",
               roll, pitch, roll_acc, pitch_acc);

        sleep_us(SAMPLE_PERIOD_US);
    }
#endif  // ENABLE_MPU6050
}