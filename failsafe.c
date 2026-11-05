#include "failsafe.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "flight_log.h"
#include "motor.h"

void failsafe(const char *reason) {
    motor_set_throttle_percent(0.0f);
    printf("FAILSAFE: %s - motors stopped\n", reason);

    // Wait indefinitely for a terminal to open the port, then dump; any keypress dumps again.
    bool was_connected = false;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !was_connected) {
            sleep_ms(500);   // let the terminal finish attaching
            flight_log_dump();
        }
        was_connected = connected;

        if (connected && getchar_timeout_us(100000) != PICO_ERROR_TIMEOUT) {
            flight_log_dump();
        } else {
            sleep_ms(100);
        }
    }
}
