#ifndef FLIGHT_PLAN_H
#define FLIGHT_PLAN_H

// Open-loop throttle profile standing in for an RC link:
// spool up -> hold hover -> spool down.
// t is seconds since the flight started. Returns the collective throttle
// in percent, or a negative value once the flight is over.
float flight_plan_throttle(float t);

#endif // FLIGHT_PLAN_H
