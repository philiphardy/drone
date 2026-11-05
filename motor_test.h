#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

// Bench test: with all motors spinning at a 14% then 18% baseline, boost each single motor,
// then GP0+GP3, then GP1+GP2 by 4% for 0.6 s, logging IMU rates, then dump the log via failsafe().
// Call after flight_controller_init() and motor_arm(). Never returns.
__attribute__((noreturn)) void motor_test_run(void);

#endif // MOTOR_TEST_H
