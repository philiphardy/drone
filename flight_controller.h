#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdbool.h>

// Bring up the attitude estimator (IMU + gyro calibration).
// Call before motor_arm(). Returns false if the IMU failed to initialize.
bool flight_controller_init(void);

// Run the fixed-rate stabilization loop: estimate attitude, run the
// roll/pitch level-hold and yaw-rate PIDs, and drive the motor mixer.
// Never returns; ends in failsafe() on IMU loss, tilt limit, or flight end.
__attribute__((noreturn)) void flight_controller_run(void);

#endif // FLIGHT_CONTROLLER_H
