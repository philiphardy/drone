#include "attitude.h"

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "kalman.h"
#include "mpu6050.h"

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define BIAS_CAL_SAMPLES 1000           // ~1 s of stationary samples

static float accel_scale;   // LSB/g
static float gyro_scale;    // LSB/(°/s)
static float yaw_bias;      // °/s; roll/pitch bias is tracked by the Kalman filters
static kalman_t roll_kf, pitch_kf;

// Convert raw accelerometer vector to roll/pitch in degrees.
// At rest, gravity points along -Z when the board is flat.
static void accel_to_angles(const vector_3d *a, float *roll, float *pitch) {
    float ax = a->x / accel_scale;
    float ay = a->y / accel_scale;
    float az = a->z / accel_scale;

    *roll  = atan2f(ay, az) * RAD_TO_DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// Average the gyro while the device is stationary to seed the bias.
// Returns bias in °/s on each axis.
static void calibrate_gyro_bias(float *bias_x, float *bias_y, float *bias_z) {
    double sx = 0, sy = 0, sz = 0;
    int n = 0;

    for (int i = 0; i < BIAS_CAL_SAMPLES; i++) {
        vector_3d g;
        if (mpu6050_read_gyro(&g)) {
            sx += g.x;
            sy += g.y;
            sz += g.z;
            n++;
        }
        sleep_us(ATTITUDE_SAMPLE_PERIOD_US);
    }

    if (n == 0) {
        *bias_x = *bias_y = *bias_z = 0.0f;
        return;
    }
    *bias_x = (float)(sx / n) / gyro_scale;
    *bias_y = (float)(sy / n) / gyro_scale;
    *bias_z = (float)(sz / n) / gyro_scale;
}

bool attitude_init(void) {
    if (!mpu6050_init()) return false;

    accel_scale = mpu6050_accel_scale();   // 16384.0f
    gyro_scale  = mpu6050_gyro_scale();    // 131.0f

    printf("MPU6050 Kalman filter — accel_scale=%.1f gyro_scale=%.3f\n",
           accel_scale, gyro_scale);
    printf("Keep the board still during bias calibration...\n");

    float bias_x, bias_y;
    calibrate_gyro_bias(&bias_x, &bias_y, &yaw_bias);
    printf("Gyro bias: x=%.3f y=%.3f z=%.3f °/s\n", bias_x, bias_y, yaw_bias);

    // Seed filters with a first accel reading so they start near reality.
    vector_3d a0, g0;
    if (mpu6050_read_sample(&a0, &g0)) {
        float r0, p0;
        accel_to_angles(&a0, &r0, &p0);
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
    return true;
}

bool attitude_update(float dt, attitude_t *out) {
    vector_3d accel, gyro;
    if (!mpu6050_read_sample(&accel, &gyro)) return false;

    float roll_acc, pitch_acc;
    accel_to_angles(&accel, &roll_acc, &pitch_acc);

    float gyro_roll  = gyro.x / gyro_scale;   // °/s
    float gyro_pitch = gyro.y / gyro_scale;

    out->roll  = kalman_update(&roll_kf,  roll_acc,  gyro_roll,  dt);
    out->pitch = kalman_update(&pitch_kf, pitch_acc, gyro_pitch, dt);

    out->roll_rate  = gyro_roll  - roll_kf.bias;
    out->pitch_rate = gyro_pitch - pitch_kf.bias;
    out->yaw_rate   = gyro.z / gyro_scale - yaw_bias;
    return true;
}
