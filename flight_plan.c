#include "flight_plan.h"

#include <math.h>

#define START_THROTTLE_PERCENT    10.0f
#define LIFTOFF_THROTTLE_PERCENT  20.0f   // legs stop chattering ~19% (flight_log_11)
#define HOVER_THROTTLE_PERCENT    24.9f   // at HOVER_REF_VOLTS: full pack (8.39 V) hovered at 24.7-25.0% (log 49)
#define HOVER_REF_VOLTS            8.4f   // resting voltage HOVER_THROTTLE_PERCENT applies to (full 2S)
#define MIN_SENSED_VOLTS           5.0f   // below this no pack is connected (USB power): no compensation
#define MAX_HOVER_PERCENT         32.0f
#define SPOOLUP_TIME_S             1.0f   // START -> LIFTOFF on the ground
#define CLIMB_RATE_M_S             0.25f
#define CLIMB_TIME_S               1.6f   // ~40 cm, plus ~5 cm overshoot into the hold (log 44)
#define HOLD_TIME_S                1.0f
#define DESCENT_RATE_M_S           0.25f
#define DESCENT_TIME_S             4.8f   // backstop only (3x nominal); the touchdown trigger normally ends the descent
// Square wave on pitch from 0.5 s after spool-up (~10 cm, out of ground effect) for 0.8 s.
#define PROBE_START_S              0.5f
#define PROBE_HALF_PERIOD_S        0.1f
#define PROBE_HALF_PERIODS         8      // whole cycles, so it adds no net tilt
#define PROBE_PERCENT              0.0f   // 0 = off; pitch identified at 0.5 (log 19)

static float hover_percent = HOVER_THROTTLE_PERCENT;

// Thrust ~ (duty * V)^2, so the hover duty scales with 1/V.
float flight_plan_set_battery_volts(float rest_volts) {
    if (rest_volts < MIN_SENSED_VOLTS) {
        hover_percent = HOVER_THROTTLE_PERCENT;
    } else {
        hover_percent = fminf(HOVER_THROTTLE_PERCENT * HOVER_REF_VOLTS / rest_volts, MAX_HOVER_PERCENT);
    }
    return hover_percent;
}

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
    sp->throttle = hover_percent;
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
