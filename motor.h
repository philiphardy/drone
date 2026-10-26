#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>

#define MOTOR_PIN 0

// Configure GP0 for 50 Hz PWM and output the idle (arming) pulse.
void motor_init(void);

// Hold the idle pulse long enough for the ESC to finish initialization.
void motor_arm(void);

// Throttle as 0-100% of the ESC's 1000-2000 us pulse range (capped at 100%).
void motor_set_throttle_percent(float percent);

#endif
