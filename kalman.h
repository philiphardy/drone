#ifndef KALMAN_H
#define KALMAN_H

#include <stdbool.h>

// One Kalman filter instance per axis (roll, pitch).
// State vector: x = [angle; bias]  (degrees, degrees/second)
typedef struct {
    float angle;        // filtered angle (degrees)
    float bias;         // estimated gyro bias (°/s)
    float P[2][2];      // error covariance matrix
} kalman_t;

// Tuning parameters. Shared by both axes; can be per-axis if you want.
typedef struct {
    float Q_angle;      // process noise: how much we distrust the gyro model
    float Q_bias;       // process noise: how much the bias is allowed to drift
    float R_measure;    // measurement noise: how much we distrust the accel
} kalman_params_t;

// Initialize a filter instance to a known starting angle.
void kalman_init(kalman_t *k, float initial_angle);

// One predict+update step.
//   accel_angle : angle from accelerometer (degrees)
//   gyro_rate   : rate from gyroscope on this axis (°/s, bias-corrected externally if desired)
//   dt          : time since last update (seconds)
// Returns the new filtered angle.
float kalman_update(kalman_t *k, float accel_angle, float gyro_rate, float dt);

// Default tuning that works well with DLPF_CFG=3 at 100 Hz.
kalman_params_t kalman_default_params(void);

// Global params used by kalman_update (set once after init).
extern kalman_params_t g_kalman_params;

#endif // KALMAN_H