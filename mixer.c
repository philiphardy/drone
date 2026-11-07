#include "mixer.h"

#include "motor.h"

// Motor index == GPIO. Axes are the ones printed on the GY-521:
//   +roll  (right-hand about +X) lifts the +Y side (GP1).
//   +pitch (right-hand about +Y) drops the +X side (GP3).
#define MOTOR_NEG_X 0
#define MOTOR_POS_Y 1
#define MOTOR_NEG_Y 2
#define MOTOR_POS_X 3

// Static per-motor thrust trim (1.0 = none); keep opposite pairs balanced so total thrust is unchanged.
// Pitch pair sized from flight_logs 11-12, roll pair from 9-12 (battery loose).
// Log 22 (battery secured): steady +0.42% u_pitch in the air => pitch pair moved 0.017.
static const float MOTOR_SCALE[4] = {
    [MOTOR_NEG_X] = 0.942f,
    [MOTOR_POS_Y] = 1.016f,
    [MOTOR_NEG_Y] = 0.984f,
    [MOTOR_POS_X] = 1.058f,
};
// +1 if the GP0/GP3 props spin clockwise seen from above, -1 if GP1/GP2 do.
#define YAW_SIGN -1.0f

// Clamp a motor command to [MIXER_IDLE_THROTTLE_PERCENT, 100].
static float clamp_throttle(float t) {
    if (t < MIXER_IDLE_THROTTLE_PERCENT) return MIXER_IDLE_THROTTLE_PERCENT;
    if (t > 100.0f) return 100.0f;
    return t;
}

void mixer_output(float throttle, float u_roll, float u_pitch, float u_yaw) {
    float yaw = YAW_SIGN * u_yaw;
    motor_set_motor_throttle_percent(MOTOR_POS_Y, clamp_throttle(throttle * MOTOR_SCALE[MOTOR_POS_Y] + u_roll  - yaw));
    motor_set_motor_throttle_percent(MOTOR_NEG_Y, clamp_throttle(throttle * MOTOR_SCALE[MOTOR_NEG_Y] - u_roll  - yaw));
    motor_set_motor_throttle_percent(MOTOR_NEG_X, clamp_throttle(throttle * MOTOR_SCALE[MOTOR_NEG_X] + u_pitch + yaw));
    motor_set_motor_throttle_percent(MOTOR_POS_X, clamp_throttle(throttle * MOTOR_SCALE[MOTOR_POS_X] - u_pitch + yaw));
}
