#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

// IMU read and control period; the MPU6050 outputs at 1 kHz (mpu6050.c), so each read is under 1 ms old.
#define ATTITUDE_SAMPLE_PERIOD_US 2000    // 500 Hz

// Axes are the ones printed on the GY-521.
typedef struct {
    float roll;         // degrees, Kalman-filtered
    float pitch;        // degrees, Kalman-filtered
    float roll_rate;    // °/s, bias-corrected
    float pitch_rate;   // °/s, bias-corrected
    float yaw_rate;     // °/s, bias-corrected
    float accel_x;      // g, mounting offset not removed
    float accel_y;      // g
    float accel_z;      // g
    float accel_up;     // g, vertical acceleration (specific force along world up - 1 g), az offset removed
} attitude_t;

// Initialize the IMU, calibrate gyro bias (board must be still for ~10 s),
// and seed the roll/pitch Kalman filters. Returns false if the IMU failed.
bool attitude_init(void);

// Read one IMU sample and advance the estimate by dt seconds. With use_accel false,
// roll/pitch integrate the gyro only (no accel correction).
// Returns false on an IMU read failure (state and *out untouched).
bool attitude_update(float dt, bool use_accel, attitude_t *out);

// Read one accelerometer sample as the IMU's own roll/pitch in degrees, before the
// mounting offset is removed (for level calibration). Returns false on an IMU read failure.
bool attitude_read_imu_angles(float *roll, float *pitch);

#endif // ATTITUDE_H
