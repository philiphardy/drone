#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

// Matches the MPU6050 output rate configured in mpu6050.c (SMPLRT_DIV=9).
#define ATTITUDE_SAMPLE_PERIOD_US 10000   // 100 Hz

// Axes are the ones printed on the GY-521.
typedef struct {
    float roll;         // degrees, Kalman-filtered
    float pitch;        // degrees, Kalman-filtered
    float roll_rate;    // °/s, bias-corrected
    float pitch_rate;   // °/s, bias-corrected
    float yaw_rate;     // °/s, bias-corrected
} attitude_t;

// Initialize the IMU, calibrate gyro bias (board must be still for ~1 s),
// and seed the roll/pitch Kalman filters. Returns false if the IMU failed.
bool attitude_init(void);

// Read one IMU sample and advance the estimate by dt seconds.
// Returns false on an IMU read failure (state and *out untouched).
bool attitude_update(float dt, attitude_t *out);

#endif // ATTITUDE_H
