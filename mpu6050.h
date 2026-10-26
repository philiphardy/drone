#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

// Raw sensor sample: signed 16-bit LSB counts per axis (big-endian in the
// MPU6050 registers, converted here to native int16_t).
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector_3d;

// Initialize I2C0 at 400 kHz on GP20 (SDA) / GP21 (SCL), reset the MPU6050,
// configure DLPF and full-scale ranges, and verify WHO_AM_I.
// Safe to call exactly once at startup, before any read.
bool mpu6050_init(void);

// Scale factors matching the full-scale ranges set by mpu6050_init().
// Defaults: ±2 g -> 16384.0f LSB/g, ±250 °/s -> 131.0f LSB/(°/s).
// Cache these once at startup; do not call inside the hot loop.
float mpu6050_accel_scale(void);
float mpu6050_gyro_scale(void);

// Read the 6 accelerometer registers (0x3B..0x40).
// Returns true on success and writes *out; false on I2C failure (out untouched).
bool mpu6050_read_acceleration(vector_3d *out);

// Read the 6 gyroscope registers (0x43..0x48).
// Returns true on success and writes *out; false on I2C failure (out untouched).
bool mpu6050_read_gyro(vector_3d *out);

// Read accel + gyro in a single 14-byte burst from 0x3B so both come from
// the same internal sample (time-coherent). Preferred for the Kalman loop.
// Returns true on success and writes both outputs; false on I2C failure
// (outputs untouched).
bool mpu6050_read_sample(vector_3d *accel, vector_3d *gyro);

// Clear a stuck I2C bus and restore the sensor configuration.
// Bit-bangs up to 9 SCL pulses to make a slave release SDA, issues a STOP,
// re-initializes the I2C peripheral, then re-applies wake + DLPF/rate/range
// settings (no DEVICE_RESET). Called automatically by the driver after an
// I2C timeout or repeated failures; may also be called manually.
// Returns true if the bus was freed and the sensor reconfigured.
bool mpu6050_recover_bus(void);

// Number of bus recoveries attempted since boot (for diagnostics).
uint32_t mpu6050_bus_recovery_count(void);

// Timeouts: every I2C transaction is bounded (3 ms), so the read functions
// above never hang. A timeout triggers recovery immediately; other failures
// (e.g. NAK) trigger it after 3 consecutive failures. The failing read still
// returns false; the next read uses the recovered bus.

#endif // MPU6050_H