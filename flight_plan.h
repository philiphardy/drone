#ifndef FLIGHT_PLAN_H
#define FLIGHT_PLAN_H

#include <stdbool.h>

// Scripted flight standing in for an RC link:
// spool up on the ground -> climb -> hold -> descend, by climb-rate setpoint.
typedef struct {
    float throttle;       // %, feedforward collective
    float climb_rate;     // m/s setpoint, up positive
    bool  velocity_loop;  // false while spooling up on the ground (climb-rate loop off)
    float pitch_probe;    // %, added to the pitch output to identify the motor response
} flight_setpoint_t;

// Scale the hover feedforward for the pack's resting voltage (measured before spool-up).
// Returns the hover throttle percent it will use.
float flight_plan_set_battery_volts(float rest_volts);

// t is seconds since the flight started. Returns false once the flight is over.
bool flight_plan_setpoint(float t, flight_setpoint_t *sp);

#endif // FLIGHT_PLAN_H
