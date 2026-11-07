#include "flight_plan.h"

#include <math.h>

#define START_THROTTLE_PERCENT    10.0f
#define LIFTOFF_THROTTLE_PERCENT  20.0f   // legs stop chattering ~19% (flight_log_11)
#define HOVER_THROTTLE_PERCENT    24.0f   // vz-loop I settled at ~+0.5 on top of 23.5 (logs 12, 16, 17)
#define SPOOLUP_TIME_S             1.0f   // START -> LIFTOFF on the ground
#define FF_RAMP_PERCENT_PER_S     10.0f   // LIFTOFF -> HOVER once the loop is on
#define CLIMB_RATE_M_S             0.25f
#define CLIMB_TIME_S               0.8f   // ~20 cm
#define HOLD_TIME_S                0.5f
#define DESCENT_RATE_M_S           0.25f
#define DESCENT_TIME_S             1.6f   // twice the nominal, so it's down before the motors cut

bool flight_plan_setpoint(float t, flight_setpoint_t *sp) {
    if (t < SPOOLUP_TIME_S) {
        sp->throttle = START_THROTTLE_PERCENT +
                       (LIFTOFF_THROTTLE_PERCENT - START_THROTTLE_PERCENT) * t / SPOOLUP_TIME_S;
        sp->climb_rate = 0.0f;
        sp->velocity_loop = false;
        return true;
    }
    t -= SPOOLUP_TIME_S;
    sp->throttle = fminf(LIFTOFF_THROTTLE_PERCENT + FF_RAMP_PERCENT_PER_S * t, HOVER_THROTTLE_PERCENT);
    sp->velocity_loop = true;
    if (t < CLIMB_TIME_S)
        sp->climb_rate = CLIMB_RATE_M_S;
    else if (t < CLIMB_TIME_S + HOLD_TIME_S)
        sp->climb_rate = 0.0f;
    else if (t < CLIMB_TIME_S + HOLD_TIME_S + DESCENT_TIME_S)
        sp->climb_rate = -DESCENT_RATE_M_S;
    else
        return false;
    return true;
}
