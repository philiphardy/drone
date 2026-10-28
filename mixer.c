#include "mixer.h"

#include "motor.h"

// Motor index == GPIO. Axes are the ones printed on the GY-521:
//   +roll  (right-hand about +X) lifts the +Y side (GP1).
//   +pitch (right-hand about +Y) drops the +X side (GP3).
#define MOTOR_NEG_X 0
#define MOTOR_POS_Y 1
#define MOTOR_NEG_Y 2
#define MOTOR_POS_X 3

// +1 if the GP0/GP3 props spin clockwise seen from above, -1 if GP1/GP2 do.
#define YAW_SIGN 1.0f

static float clamp_throttle(float t) {
    if (t < MIXER_IDLE_THROTTLE_PERCENT) return MIXER_IDLE_THROTTLE_PERCENT;
    if (t > 100.0f) return 100.0f;
    return t;
}

void mixer_output(float throttle, float u_roll, float u_pitch, float u_yaw) {
    float yaw = YAW_SIGN * u_yaw;
    motor_set_motor_throttle_percent(MOTOR_POS_Y, clamp_throttle(throttle + u_roll  - yaw));
    motor_set_motor_throttle_percent(MOTOR_NEG_Y, clamp_throttle(throttle - u_roll  - yaw));
    motor_set_motor_throttle_percent(MOTOR_NEG_X, clamp_throttle(throttle + u_pitch + yaw));
    motor_set_motor_throttle_percent(MOTOR_POS_X, clamp_throttle(throttle - u_pitch + yaw));
}
