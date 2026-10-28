#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>

#define MOTOR_COUNT 4
#define MOTOR_FIRST_PIN 0   // motors 0-3 are on GP0-GP3
#define MOTOR_ENABLED_MASK 0xE  // bit n = motor n gets DSHOT frames; motor 0 (GP0) is unplugged

// Start DSHOT on GP0-GP3 (PIO0, one state machine each; sent from core 1) and begin sending zero throttle.
void motor_init(void);

// Hold zero throttle long enough for the ESCs to finish initialization.
void motor_arm(void);

// Throttle for one motor (0 to MOTOR_COUNT-1) as 0-100% (capped at 100%).
// 0% is stopped (DSHOT 0); above 0% maps to DSHOT 48-2047.
void motor_set_motor_throttle_percent(unsigned motor, float percent);

// Same throttle for all motors.
void motor_set_throttle_percent(float percent);

#endif
