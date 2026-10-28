#include "failsafe.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"

void failsafe(const char *reason) {
    motor_set_throttle_percent(0.0f);
    printf("FAILSAFE: %s - motors stopped\n", reason);
    while (true) sleep_ms(1000);
}
