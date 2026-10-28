#include "motor_test.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"

void motor_test_run(void) {
    for (int percent = 10; percent <= 20; percent += 5) {
        for (unsigned m = 1; m < MOTOR_COUNT; m++) {
            motor_set_motor_throttle_percent(m, (float)percent);
        }
        printf("Motors 1-3 throttle %d%%\n", percent);
        sleep_ms(percent == 20 ? 3000 : 1000);
    }
    motor_set_throttle_percent(0.0f);
    printf("Motors 1-3 throttle 0%%\n");
    while (true) sleep_ms(1000);
}
