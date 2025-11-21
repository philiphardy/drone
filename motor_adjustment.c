#include <stdio.h>
#include <math.h>
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "pico/mem_ops.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

// TODO: Reduce max power to 90% to avoid burning out motors since 4.5v battery supply will be used
#define MAX_POWER 100
#define MAX_ANGLE 90

#define NUM_MOTORS 4

#define FRONT_MOTOR_PIN 22
#define BACK_MOTOR_PIN 17
#define LEFT_MOTOR_PIN 14
#define RIGHT_MOTOR_PIN 10

static void task_entry();
static void setup_pwm();
static void adjust_pwm(uint32_t pin, float power);
static void adjust_front_motor(float pitch);
static void adjust_back_motor(float pitch);
static void adjust_left_motor(float roll);
static void adjust_right_motor(float roll);
static float normalize_power(float p);

// TODO: Calculate at task startup after calibration has been completed in flight control task
static float hovering_power = 50.0f;

/**
 * motor_adjustment_init
 *
 * Launch the motor adjustment task on core 1. The task listens for pitch
 * and roll values on the multicore FIFO and updates per-motor power levels.
 */
void motor_adjustment_init() {
    setup_pwm();

    // start task on core 1
    multicore_launch_core1(task_entry);
}

/**
 * setup_pwm
 *
 * Configure the motor GPIOs for PWM output and initialize the
 * PWM slice. The function sets the PWM wrap value to `MAX_POWER - 1` so
 * that `power` values in the range [0, MAX_POWER] map directly to the
 * channel level (after casting).
 */
static void setup_pwm() {
    bi_decl(bi_1pin_with_func(FRONT_MOTOR_PIN, GPIO_FUNC_PWM));
    bi_decl(bi_1pin_with_func(BACK_MOTOR_PIN, GPIO_FUNC_PWM));
    bi_decl(bi_1pin_with_func(LEFT_MOTOR_PIN, GPIO_FUNC_PWM));
    bi_decl(bi_1pin_with_func(RIGHT_MOTOR_PIN, GPIO_FUNC_PWM));

    uint32_t motor_pins[] = {FRONT_MOTOR_PIN, BACK_MOTOR_PIN, LEFT_MOTOR_PIN, RIGHT_MOTOR_PIN};
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        uint32_t pin = motor_pins[i];

        gpio_set_function(pin, GPIO_FUNC_PWM);

        uint32_t slice = pwm_gpio_to_slice_num(pin);
        uint32_t channel = pwm_gpio_to_channel(pin);
    
        pwm_set_wrap(slice, MAX_POWER-1);
        pwm_set_chan_level(slice, channel, 0);
        pwm_set_enabled(slice, true);
    }
}

/**
 * adjust_pwm
 *
 * Set the PWM duty level for a motor pin.
 *
 * @param pin GPIO pin number configured for PWM
 * @param power Duty level expressed in the same units as the PWM wrap
 *              (0 .. MAX_POWER). The value is cast to `uint16_t` before
 *              being written to the PWM channel.
 */
static void adjust_pwm(uint32_t pin, float power) {
    uint32_t slice = pwm_gpio_to_slice_num(pin);
    uint32_t channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, (uint16_t)power);
    printf("Power: %d\n", (uint16_t)power);
}

/**
 * task_entry
 *
 * Core1 entrypoint: blocks on the multicore FIFO for two 32-bit words
 * representing the current `pitch` and `roll` (in that order). Each value
 * is sent as the float bit-pattern cast to `uint32_t` on the producing core.
 *
 * When a pair is received the task computes and applies per-motor power
 * adjustments (here represented by internal variables and debug prints).
 */
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

/**
 * adjust_front_motor
 *
 * Compute and set the front motor power based on `pitch` (degrees).
 * - `pitch` is expected in degrees.
 * - Uses `MAX_ANGLE` to scale the control multiplier.
 */
static void adjust_front_motor(float pitch) {
    float multiplier = fabsf(pitch / MAX_ANGLE);
    if (pitch < 0) {
        // front is pitching down -- increase motor power
        multiplier += 1;
    } else {
        // front is pitching up -- decrease motor power
        multiplier = 1 - multiplier;
    }
    float motor_power = normalize_power(hovering_power * multiplier);
    printf("Front ");
    adjust_pwm(FRONT_MOTOR_PIN, motor_power);
}

/**
 * adjust_back_motor
 *
 * Compute and set the back motor power based on `pitch` (degrees).
 */
static void adjust_back_motor(float pitch) {
    float multiplier = fabsf(pitch / MAX_ANGLE);
    if (pitch > 0) {
        // back is pitching down -- increase motor power
        multiplier += 1;
    } else {
        // back is pitching up -- decrease motor power
        multiplier = 1 - multiplier;
    }
    float motor_power = normalize_power(hovering_power * multiplier);
    printf("Back ");
    adjust_pwm(BACK_MOTOR_PIN, motor_power);
}

/**
 * adjust_left_motor
 *
 * Compute and set the left motor power based on `roll` (degrees).
 */
static void adjust_left_motor(float roll) {
    float multiplier = fabsf(roll / MAX_ANGLE);
    if (roll < 0) {
        // left is pitching down -- increase motor power
        multiplier += 1;
    } else {
        // left is pitching up -- decrease motor power
        multiplier = 1 - multiplier;
    }
    float motor_power = normalize_power(hovering_power * multiplier);
    printf("Left ");
    adjust_pwm(LEFT_MOTOR_PIN, motor_power);
}

/**
 * adjust_right_motor
 *
 * Compute and set the right motor power based on `roll` (degrees).
 */
static void adjust_right_motor(float roll) {
    float multiplier = fabsf(roll / MAX_ANGLE);
    if (roll > 0) {
        // right is pitching down -- decrease motor power
        multiplier += 1;
    } else {
        // right is pitching up -- decrease motor power
        multiplier = 1 - multiplier;
    }
    float motor_power = normalize_power(hovering_power * multiplier);
    printf("Right ");
    adjust_pwm(RIGHT_MOTOR_PIN, motor_power);
}

/**
 * normalize_power
 *
 * Clamp motor power to the valid range [0, MAX_POWER].
 *
 * @param p Desired power value (float)
 * @return Clamped power value (float)
 */
static float normalize_power(float p) {
    p = MAX(0, p);
    p = MIN(MAX_POWER, p);
    return p;
}
