#include "motor.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "dshot_tx.pio.h"

#define DSHOT_SPEED_KBPS     300u    // DSHOT300
#define DSHOT_THROTTLE_MIN   48u     // 0 = disarmed, 1-47 are reserved for commands
#define DSHOT_THROTTLE_MAX   2047u
#define FRAME_PERIOD_US      500u    // 2 kHz frame rate
#define ARM_HOLD_MS          3000u
#define MAX_THROTTLE_PERCENT 100.0f  // hard cap applied to every throttle command

static PIO motor_pio = pio0;
static uint motor_sm[MOTOR_COUNT];

// Written by core 0, consumed by core 1.
static volatile uint16_t target_throttle[MOTOR_COUNT];

// Build a 16-bit DSHOT frame: 11-bit throttle, telemetry bit (0), 4-bit CRC.
static uint16_t dshot_frame(uint16_t throttle) {
    uint16_t value = (uint16_t)(throttle << 1);   // telemetry bit = 0
    uint16_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    return (uint16_t)((value << 4) | crc);
}

// ESCs expect a continuous stream of frames, so core 1 resends the latest targets.
static void dshot_core1_entry(void) {
    while (true) {
        for (unsigned i = 0; i < MOTOR_COUNT; i++) {
            if (!(MOTOR_ENABLED_MASK & (1u << i))) continue;
            pio_sm_put_blocking(motor_pio, motor_sm[i],
                                (uint32_t)dshot_frame(target_throttle[i]) << 16);
        }
        sleep_us(FRAME_PERIOD_US);
    }
}

void motor_init(void) {
    uint offset = pio_add_program(motor_pio, &dshot_tx_program);
    for (unsigned i = 0; i < MOTOR_COUNT; i++) {
        target_throttle[i] = 0;
        motor_sm[i] = pio_claim_unused_sm(motor_pio, true);
        dshot_tx_program_init(motor_pio, motor_sm[i], offset, MOTOR_FIRST_PIN + i, DSHOT_SPEED_KBPS);
    }
    multicore_launch_core1(dshot_core1_entry);
}

void motor_arm(void) {
    motor_set_throttle_percent(0.0f);
    sleep_ms(ARM_HOLD_MS);
}

void motor_set_motor_throttle_percent(unsigned motor, float percent) {
    if (motor >= MOTOR_COUNT) return;
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > MAX_THROTTLE_PERCENT) percent = MAX_THROTTLE_PERCENT;
    // 0% means stopped (DSHOT 0), not the minimum throttle step.
    if (percent == 0.0f) {
        target_throttle[motor] = 0;
        return;
    }
    // +0.5 rounds to the nearest step so float error doesn't truncate down.
    target_throttle[motor] = (uint16_t)(DSHOT_THROTTLE_MIN +
                             (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) * percent / 100.0f + 0.5f);
}

void motor_set_throttle_percent(float percent) {
    for (unsigned i = 0; i < MOTOR_COUNT; i++) {
        motor_set_motor_throttle_percent(i, percent);
    }
}
