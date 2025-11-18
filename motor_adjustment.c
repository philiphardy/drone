#include <stdio.h>
#include <math.h>
#include "pico/mem_ops.h"
#include "pico/multicore.h"

#define MAX_POWER 100
#define MAX_ANGLE 90

static void task_entry();
static void adjust_front_motor(float pitch);
static void adjust_back_motor(float pitch);
static void adjust_left_motor(float roll);
static void adjust_right_motor(float roll);
static float normalize_power(float p);

static float hovering_power;

void motor_adjustment_init() {
    // start task on core 1
    multicore_launch_core1(task_entry);
}

static void task_entry() {
    while (true) {
        uint32_t msg = multicore_fifo_pop_blocking();
        float pitch = *(float *)&msg;
        msg = multicore_fifo_pop_blocking();
        float roll = *(float *)&msg;
        printf("Pitch: %.2f  Roll: %.2f\n", pitch, roll);

        adjust_front_motor(pitch);
        adjust_back_motor(pitch);
        adjust_left_motor(roll);
        adjust_right_motor(roll);
    }
}

static float front_motor_power;
static void adjust_front_motor(float pitch) {
    float multiplier = fabsf(pitch / MAX_ANGLE);
    if (pitch < 0) {
        // nose is pitching down -- increase motor power
        multiplier += 1;
    } else {
        // nose is pitching up -- decrease motor power
        multiplier = 1 - multiplier;
    }
    front_motor_power = hovering_power * multiplier;
    front_motor_power = normalize_power(front_motor_power);
    printf("FP: %.2f\n", multiplier);
}

static float back_motor_power;
static void adjust_back_motor(float pitch) {
    float multiplier = fabsf(pitch / MAX_ANGLE);
    if (pitch > 0) {
        // rear is pitching down -- increase motor power
        multiplier += 1;
    } else {
        // rear is pitching up -- decrease motor power
        multiplier = 1 - multiplier;
    }
    back_motor_power = hovering_power * back_motor_power;
    back_motor_power = normalize_power(back_motor_power);
    printf("BP: %.2f\n", multiplier);
}

static float left_motor_power;
static void adjust_left_motor(float roll) {
    float multiplier = fabsf(roll / MAX_ANGLE);
    if (roll < 0) {
        // rear is pitching down -- increase motor power
        multiplier += 1;
    } else {
        // rear is pitching up -- decrease motor power
        multiplier = 1 - multiplier;
    }
    left_motor_power = hovering_power * multiplier;
    left_motor_power = normalize_power(left_motor_power);
    printf("LP: %.2f\n", multiplier);
}

static float right_motor_power;
static void adjust_right_motor(float roll) {
    float multiplier = fabsf(roll / MAX_ANGLE);
    if (roll > 0) {
        multiplier += 1;
    } else {
        // rear is pitching up -- decrease motor power
        float multiplier = 1 - multiplier;
    }
    right_motor_power = hovering_power * multiplier;
    right_motor_power = normalize_power(right_motor_power);
    printf("RP: %.2f\n", multiplier);
}

static float normalize_power(float p) {
    p = MAX(0, p);
    p = MIN(MAX_POWER, p);
    return p;
}
