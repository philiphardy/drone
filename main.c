#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"

// 1 = flight controller, 0 = motor ramp test only.
#define ENABLE_MPU6050 1

#if ENABLE_MPU6050
#include "flight_controller.h"
#else
#include "motor_test.h"
#endif

// Bring up stdio and motors, arm the ESCs, then hand off to the flight
// controller (or motor test). Never returns.
int main(void) {
    stdio_init_all();

    // Let the USB CDC serial port enumerate before we print.
    sleep_ms(5000);

    // Start sending the idle pulse immediately so the ESC can initialize.
    motor_init();

#if ENABLE_MPU6050
    if (!flight_controller_init()) {
        printf("MPU6050 init failed â€” halting\n");
        while (true) sleep_ms(1000);
    }
#endif

    printf("Arming ESC...\n");
    motor_arm();

#if ENABLE_MPU6050
    flight_controller_run();
#else
    motor_test_run();
#endif
}
