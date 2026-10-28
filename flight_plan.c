#include "flight_plan.h"

#define HOVER_THROTTLE_PERCENT  30.0f   // tune until the drone just holds altitude
#define SPOOLUP_TIME_S           3.0f
#define FLIGHT_TIME_S           10.0f   // no RC link, so the flight ends on a timer
#define SPOOLDOWN_TIME_S         3.0f

float flight_plan_throttle(float t) {
    if (t < SPOOLUP_TIME_S) return HOVER_THROTTLE_PERCENT * t / SPOOLUP_TIME_S;
    if (t < FLIGHT_TIME_S)  return HOVER_THROTTLE_PERCENT;
    float down = t - FLIGHT_TIME_S;
    if (down < SPOOLDOWN_TIME_S)
        return HOVER_THROTTLE_PERCENT * (1.0f - down / SPOOLDOWN_TIME_S);
    return -1.0f;
}
