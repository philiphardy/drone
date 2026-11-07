#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"

// 1 = per-pair motor test (logs IMU rates, then dumps), 0 = flight controller.
#define RUN_MOTOR_TEST 0
// 1 = IMU level calibration: motors stay disarmed, offsets print over USB. Overrides RUN_MOTOR_TEST.
#define RUN_IMU_CAL 0

#include "flight_controller.h"
#include "imu_cal.h"
#include "motor_test.h"

// Bring up stdio and motors, arm the ESCs, then hand off to the flight
// controller (or motor test). The IMU calibration runs before arming. Never returns.
int main(void) {
    stdio_init_all();

    // Let the USB CDC serial port enumerate before we print.
    sleep_ms(2000);

    // Start sending the idle pulse immediately so the ESC can initialize.
    motor_init();

    if (!flight_controller_init()) {
        printf("MPU6050 init failed â€” halting\n");
        while (true) sleep_ms(1000);
    }

#if RUN_IMU_CAL
    imu_cal_run();
#else
    printf("Arming ESC...\n");
    motor_arm();

    #if RUN_MOTOR_TEST
    motor_test_run();
    #else
    flight_controller_run();
    #endif
#endif

}
