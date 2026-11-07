#include "attitude.h"

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "kalman.h"
#include "mpu6050.h"

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define BIAS_CAL_SAMPLES   1000         // ~2 s of stationary samples
#define BIAS_CAL_PERIOD_US 2000
#define SEED_ANGLE_VAR   0.01f          // deg², one still accel sample
#define SEED_BIAS_VAR    1e-4f          // (deg/s)², averaged still gyro
// IMU roll/pitch with the frame level on a level surface; measure with RUN_IMU_CAL in main.c.
#define IMU_ROLL_OFFSET_DEG  -0.41f
#define IMU_PITCH_OFFSET_DEG  2.83f

static float accel_scale;   // LSB/g
static float gyro_scale;    // LSB/(°/s)
static float yaw_bias;      // °/s; roll/pitch bias is tracked by the Kalman filters
static float az_offset;     // g; this MPU6050 reads ~+0.19 g high on Z
static kalman_t roll_kf, pitch_kf;

// Convert raw accelerometer vector to the IMU's own roll/pitch in degrees.
// At rest, gravity points along -Z when the board is flat.
static void accel_to_angles(const vector_3d *a, float *roll, float *pitch) {
    float ax = a->x / accel_scale;
    float ay = a->y / accel_scale;
    float az = a->z / accel_scale;

    *roll  = atan2f(ay, az) * RAD_TO_DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// Frame roll/pitch: the IMU's angles minus its mounting tilt.
static void accel_to_frame_angles(const vector_3d *a, float *roll, float *pitch) {
    accel_to_angles(a, roll, pitch);
    *roll  -= IMU_ROLL_OFFSET_DEG;
    *pitch -= IMU_PITCH_OFFSET_DEG;
}

// Average the IMU while the device is stationary to seed the gyro bias (°/s on each axis)
// and az_offset (the Z reading beyond what gravity explains, given X and Y).
static void calibrate_at_rest(float *bias_x, float *bias_y, float *bias_z) {
    double sx = 0, sy = 0, sz = 0, sax = 0, say = 0, saz = 0;
    int n = 0;

    for (int i = 0; i < BIAS_CAL_SAMPLES; i++) {
        vector_3d a, g;
        if (mpu6050_read_sample(&a, &g)) {
            sx += g.x;
            sy += g.y;
            sz += g.z;
            sax += a.x;
            say += a.y;
            saz += a.z;
            n++;
        }
        sleep_us(BIAS_CAL_PERIOD_US);
    }

    if (n == 0) {
        *bias_x = *bias_y = *bias_z = 0.0f;
        az_offset = 0.0f;
        return;
    }
    *bias_x = (float)(sx / n) / gyro_scale;
    *bias_y = (float)(sy / n) / gyro_scale;
    *bias_z = (float)(sz / n) / gyro_scale;
    float ax = (float)(sax / n) / accel_scale;
    float ay = (float)(say / n) / accel_scale;
    az_offset = (float)(saz / n) / accel_scale - sqrtf(fmaxf(0.0f, 1.0f - ax * ax - ay * ay));
}

bool attitude_init(void) {
    if (!mpu6050_init()) return false;

    accel_scale = mpu6050_accel_scale();   // 16384.0f
    gyro_scale  = mpu6050_gyro_scale();    // 65.5f

    printf("MPU6050 Kalman filter — accel_scale=%.1f gyro_scale=%.3f\n",
           accel_scale, gyro_scale);
    printf("Keep the board still during bias calibration...\n");

    float bias_x, bias_y;
    calibrate_at_rest(&bias_x, &bias_y, &yaw_bias);
    printf("Gyro bias: x=%.3f y=%.3f z=%.3f °/s, accel Z offset %+.3f g\n", bias_x, bias_y, yaw_bias, az_offset);

    // Seed filters with a first accel reading so they start near reality.
    vector_3d a0, g0;
    if (mpu6050_read_sample(&a0, &g0)) {
        float r0, p0;
        accel_to_frame_angles(&a0, &r0, &p0);
        kalman_init(&roll_kf,  r0);
        kalman_init(&pitch_kf, p0);
        // Seed bias estimates directly so the filter doesn't spend
        // hundreds of milliseconds learning what we already know.
        roll_kf.bias  = bias_x;
        pitch_kf.bias = bias_y;
        // Unit covariance let spin-up accel noise rewrite the bias by ~50 °/s.
        roll_kf.P[0][0] = pitch_kf.P[0][0] = SEED_ANGLE_VAR;
        roll_kf.P[1][1] = pitch_kf.P[1][1] = SEED_BIAS_VAR;
    } else {
        kalman_init(&roll_kf,  0.0f);
        kalman_init(&pitch_kf, 0.0f);
    }
    return true;
}

bool attitude_update(float dt, bool use_accel, attitude_t *out) {
    vector_3d accel, gyro;
    if (!mpu6050_read_sample(&accel, &gyro)) return false;

    float roll_acc, pitch_acc;
    accel_to_frame_angles(&accel, &roll_acc, &pitch_acc);

    float gyro_roll  = gyro.x / gyro_scale;   // °/s
    float gyro_pitch = gyro.y / gyro_scale;

    if (use_accel) {
        out->roll  = kalman_update(&roll_kf,  roll_acc,  gyro_roll,  dt);
        out->pitch = kalman_update(&pitch_kf, pitch_acc, gyro_pitch, dt);
    } else {
        out->roll  = roll_kf.angle  += (gyro_roll  - roll_kf.bias)  * dt;
        out->pitch = pitch_kf.angle += (gyro_pitch - pitch_kf.bias) * dt;
    }

    out->roll_rate  = gyro_roll  - roll_kf.bias;
    out->pitch_rate = gyro_pitch - pitch_kf.bias;
    out->yaw_rate   = gyro.z / gyro_scale - yaw_bias;
    out->accel_x    = accel.x / accel_scale;
    out->accel_y    = accel.y / accel_scale;
    out->accel_z    = accel.z / accel_scale;

    // World up in IMU axes, from the IMU's own tilt (frame angle + mounting offset).
    float ph = (out->roll  + IMU_ROLL_OFFSET_DEG)  / RAD_TO_DEG;
    float th = (out->pitch + IMU_PITCH_OFFSET_DEG) / RAD_TO_DEG;
    float az = out->accel_z - az_offset;
    out->accel_up = -out->accel_x * sinf(th)
                  + (out->accel_y * sinf(ph) + az * cosf(ph)) * cosf(th) - 1.0f;
    return true;
}

bool attitude_read_imu_angles(float *roll, float *pitch) {
    vector_3d a;
    if (!mpu6050_read_acceleration(&a)) return false;
    accel_to_angles(&a, roll, pitch);
    return true;
}
