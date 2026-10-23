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

#endif // MPU6050_H