#include "flight_plan.h"

#define START_THROTTLE_PERCENT  10.0f
#define HOVER_THROTTLE_PERCENT  24.0f   // tune until the drone just holds altitude
#define SPOOLUP_TIME_S           1.0f
#define HOVER_TIME_S             4.0f
#define FLIGHT_TIME_S           (SPOOLUP_TIME_S + HOVER_TIME_S)   // no RC link, so the flight ends on a timer
#define SPOOLDOWN_TIME_S         0.5f

float flight_plan_throttle(float t) {
    if (t < SPOOLUP_TIME_S)
        return START_THROTTLE_PERCENT +
               (HOVER_THROTTLE_PERCENT - START_THROTTLE_PERCENT) * t / SPOOLUP_TIME_S;
    if (t < FLIGHT_TIME_S)  return HOVER_THROTTLE_PERCENT;
    float down = t - FLIGHT_TIME_S;
    if (down < SPOOLDOWN_TIME_S)
        return HOVER_THROTTLE_PERCENT * (1.0f - down / SPOOLDOWN_TIME_S);
    return -1.0f;
}
