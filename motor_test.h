#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

// Bench test without the IMU: ramp GP1-3 from 10% to 20% in 5% steps
// (1 s each, 3 s at 20%), then stop the motors and halt.
// Call after motor_arm(). Never returns.
__attribute__((noreturn)) void motor_test_run(void);

#endif // MOTOR_TEST_H
