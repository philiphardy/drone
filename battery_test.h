#ifndef BATTERY_TEST_H
#define BATTERY_TEST_H

// Bench test (motors never armed): print the battery voltage over USB every 500 ms. Never returns.
__attribute__((noreturn)) void battery_test_run(void);

#endif // BATTERY_TEST_H
