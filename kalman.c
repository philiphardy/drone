#include "kalman.h"

// Defaults match the sensor configuration in mpu6050.c:
//   DLPF_CFG = 3 (44 Hz / 42 Hz bandwidth), 100 Hz sample rate.
// Tune by changing exactly one parameter at a time while observing the
// step response and steady-state noise.
kalman_params_t g_kalman_params = {
    .Q_angle   = 0.001f,
    .Q_bias    = 0.003f,
    .R_measure = 0.03f,
};

kalman_params_t kalman_default_params(void) {
    return g_kalman_params;
}

void kalman_init(kalman_t *k, float initial_angle) {
    k->angle    = initial_angle;
    k->bias     = 0.0f;
    k->P[0][0]  = 1.0f;
    k->P[0][1]  = 0.0f;
    k->P[1][0]  = 0.0f;
    k->P[1][1]  = 1.0f;
}

float kalman_update(kalman_t *k, float accel_angle, float gyro_rate, float dt) {
    // ── 1. Predict ────────────────────────────────────────────────
    // x' = F x + B u   with  F = [[1, -dt], [0, 1]],  B = [dt, 0]^T
    float rate = gyro_rate - k->bias;
    k->angle += dt * rate;

    // P' = F P F^T + Q  (expanded for the 2x2 case)
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0]
                        + g_kalman_params.Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += g_kalman_params.Q_bias * dt;

    // ── 2. Innovation ─────────────────────────────────────────────
    // H = [1, 0], so  y = z - H x  =  accel_angle - angle
    float y = accel_angle - k->angle;
    float S = k->P[0][0] + g_kalman_params.R_measure;   // innovation covariance

    // ── 3. Kalman gain  K = P H^T / S  (H picks column 0 of P) ────
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    // ── 4. State update ──────────────────────────────────────────
    k->angle += K0 * y;
    k->bias  += K1 * y;

    // ── 5. Covariance update  P = (I - K H) P ─────────────────────
    float P00 = k->P[0][0];
    float P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}