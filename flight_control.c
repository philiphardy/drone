#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "mpu6050.h"
#include "motor_adjustment.h"

#define NUM_CAL_SAMPLES 10
#define MS_PER_SEC 1000
#define CAL_TIME_MS (1 * MS_PER_SEC)
#define CAL_SAMPLE_SLEEP_MS (CAL_TIME_MS / NUM_CAL_SAMPLES)

static void flight_control_calibrate();
static void flight_control_begin();
static void normalize_vector(vector_3d * v);
static void notify_motor_adjustment_of_accel(vector_3d * accel);

static vector_3d calibratedAccel;
static vector_3d prevAccel;

void flight_control_init() {
    mpu6050_init();
    motor_adjustment_init();
    flight_control_calibrate();
    flight_control_begin();
}

static void flight_control_calibrate() {
    int32_t accelXSum, accelYSum, accelZSum;
    accelXSum = accelYSum = accelZSum = 0;

    for (uint8_t i = 0; i < NUM_CAL_SAMPLES; i++) {
        vector_3d v = mpu6050_read_acceleration();
        accelXSum += v.x;
        accelYSum += v.y;
        accelZSum += v.z;
        sleep_ms(CAL_SAMPLE_SLEEP_MS);
    }

    calibratedAccel = (vector_3d) {
        .x = (accelXSum / NUM_CAL_SAMPLES),
        .y = (accelYSum / NUM_CAL_SAMPLES),
        .z = (accelZSum / NUM_CAL_SAMPLES)
    };
}

static void flight_control_begin() {
    while (true) {
        printv("Calibrated", &calibratedAccel);
        vector_3d currAccel = mpu6050_read_acceleration();
        normalize_vector(&currAccel);
        printv("Current Accel", &currAccel);

        vector_3d deltaAccel = (vector_3d) {
            .x = currAccel.x - prevAccel.x,
            .y = currAccel.y - prevAccel.y,
            .z = currAccel.z - prevAccel.z,
        };
        printv("Delta Accel", &deltaAccel);

        prevAccel = currAccel;

        // Pass current and delta acceleration to the motor adjustment task
        notify_motor_adjustment_of_accel(&currAccel);
        notify_motor_adjustment_of_accel(&deltaAccel);
        sleep_ms(1000);
    }
}

static void normalize_vector(vector_3d * v) {
    v->x = v->x - calibratedAccel.x;
    v->y = v->y - calibratedAccel.y;
    v->z = v->z - calibratedAccel.z;
}

static void notify_motor_adjustment_of_accel(vector_3d * accel) {
    multicore_fifo_push_blocking(accel->x);
    multicore_fifo_push_blocking(accel->y);
    multicore_fifo_push_blocking(accel->z);
}