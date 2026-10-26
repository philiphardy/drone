#include "motor.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define PWM_PERIOD_US   20000u   // 50 Hz
#define PULSE_MIN_US    1000u    // idle / init signal (spec: 800-1050 us)
#define PULSE_MAX_US    2000u    // full throttle (spec: start range 1050-2000 us)
#define ARM_HOLD_MS     3000u
#define MAX_THROTTLE_PERCENT 100.0f  // hard cap applied to every throttle command

static void set_pulse_us(uint32_t us) {
    // With a 1 MHz PWM tick, the channel level equals the pulse width in us.
    pwm_set_gpio_level(MOTOR_PIN, us);
}

void motor_init(void) {
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_PIN);

    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_set_wrap(slice, PWM_PERIOD_US - 1);
    set_pulse_us(PULSE_MIN_US);
    pwm_set_enabled(slice, true);
}

void motor_arm(void) {
    set_pulse_us(PULSE_MIN_US);
    sleep_ms(ARM_HOLD_MS);
}

void motor_set_throttle_percent(float percent) {
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > MAX_THROTTLE_PERCENT) percent = MAX_THROTTLE_PERCENT;
    // +0.5 rounds to the nearest us so float error doesn't truncate 41.1% down a step.
    set_pulse_us(PULSE_MIN_US +
                 (uint32_t)((PULSE_MAX_US - PULSE_MIN_US) * percent / 100.0f + 0.5f));
}
