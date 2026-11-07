#include "flight_plan.h"

#define START_THROTTLE_PERCENT  10.0f
#define PEAK_THROTTLE_PERCENT   26.0f   // hover is ~22-23% (flight_log_9)
#define SPOOLUP_TIME_S           1.5f
#define SPOOLDOWN_TIME_S         2.0f

float flight_plan_throttle(float t) {
    if (t < SPOOLUP_TIME_S)
        return START_THROTTLE_PERCENT +
               (PEAK_THROTTLE_PERCENT - START_THROTTLE_PERCENT) * t / SPOOLUP_TIME_S;
    float down = t - SPOOLUP_TIME_S;
    if (down < SPOOLDOWN_TIME_S)
        return PEAK_THROTTLE_PERCENT * (1.0f - down / SPOOLDOWN_TIME_S);
    return -1.0f;
}
