#include "flight_plan.h"

#include <math.h>

#define START_THROTTLE_PERCENT    10.0f
#define LIFTOFF_THROTTLE_PERCENT  20.0f   // legs stop chattering ~19% (flight_log_11)
#define HOVER_THROTTLE_PERCENT    24.0f   // vz-loop I settled at ~+0.5 on top of 23.5 (logs 12, 16, 17)
#define SPOOLUP_TIME_S             1.0f   // START -> LIFTOFF on the ground
#define CLIMB_RATE_M_S             0.25f
#define CLIMB_TIME_S               0.8f   // ~20 cm
#define HOLD_TIME_S                0.5f
#define DESCENT_RATE_M_S           0.25f
#define DESCENT_TIME_S             1.6f   // twice the nominal, so it's down before the motors cut
// Square wave on pitch from mid-climb (~10 cm, out of ground effect) to the end of the hold.
#define PROBE_START_S              0.5f
#define PROBE_HALF_PERIOD_S        0.1f
#define PROBE_HALF_PERIODS         8      // whole cycles, so it adds no net tilt
#define PROBE_PERCENT              0.0f   // 0 = off; pitch identified at 0.5 (log 19)

bool flight_plan_setpoint(float t, flight_setpoint_t *sp) {
    sp->pitch_probe = 0.0f;
    if (t < SPOOLUP_TIME_S) {
        sp->throttle = START_THROTTLE_PERCENT +
                       (LIFTOFF_THROTTLE_PERCENT - START_THROTTLE_PERCENT) * t / SPOOLUP_TIME_S;
        sp->climb_rate = 0.0f;
        sp->velocity_loop = false;
        return true;
    }
    t -= SPOOLUP_TIME_S;
    // Step, not ramp: a slow ramp leaves it pivoting on its last legs for ~0.3 s (logs 22-24).
    sp->throttle = HOVER_THROTTLE_PERCENT;
    sp->velocity_loop = true;
    int half = (int)floorf((t - PROBE_START_S) / PROBE_HALF_PERIOD_S);
    if (t >= PROBE_START_S && half < PROBE_HALF_PERIODS)
        sp->pitch_probe = (half % 2 == 0) ? PROBE_PERCENT : -PROBE_PERCENT;
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
