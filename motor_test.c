#include "motor_test.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"

void motor_test_run(void) {
    for (unsigned m = 0; m < MOTOR_COUNT; m++) {
        motor_set_motor_throttle_percent(m, 1.0f);
    }
    printf("All motors throttle 1%%\n");
    sleep_ms(10000);
    motor_set_throttle_percent(0.0f);
    printf("All motors throttle 0%%\n");
    while (true) sleep_ms(1000);
}
