#include "pid.h"

static float clampf(float v, float lim) {
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

void pid_reset(pid_ctrl_t *p) {
    p->integral = 0.0f;
}

float pid_update(pid_ctrl_t *p, float setpoint, float measurement,
                 float measurement_rate, float dt, bool integrate) {
    float error = setpoint - measurement;

    if (integrate) {
        p->integral = clampf(p->integral + p->ki * error * dt, p->i_limit);
    }

    float out = p->kp * error + p->integral - p->kd * measurement_rate;
    return clampf(out, p->out_limit);
}
