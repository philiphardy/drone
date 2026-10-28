#ifndef PID_H
#define PID_H

#include <stdbool.h>

// Named pid_ctrl_t because newlib already defines pid_t (process id).
typedef struct {
    float kp, ki, kd;
    float i_limit;      // clamp on the integral term (output units)
    float out_limit;    // clamp on the total output (output units)
    float integral;     // accumulated ki * error * dt
} pid_ctrl_t;

// Clear the accumulated integral term; gains and limits are kept.
void pid_reset(pid_ctrl_t *p);

// Run one PID step and return kp*error + integral - kd*measurement_rate,
// clamped to ±out_limit (integral itself clamped to ±i_limit).
// D acts on measurement_rate (e.g. gyro rate) rather than the differentiated
// error, so there is no derivative kick and no noisy numerical derivative.
// Pass integrate = false to freeze the integral (e.g. while on the ground).
float pid_update(pid_ctrl_t *p, float setpoint, float measurement,
                 float measurement_rate, float dt, bool integrate);

#endif // PID_H
