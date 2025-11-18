/**
 * Copyright Phil Hardy 2025
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

// By default these devices are on bus address 0x68
#define ADDR 0x68
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 20

static void mpu6050_reset();

#ifdef i2c_default
    /**
     * mpu6050_init
     *
     * Initialize I2C and configure the MPU6050 device. This sets up I2C0
     * at 400kHz on the configured SDA/SCL pins, enables pull-ups, and calls
     * `mpu6050_reset()` to ensure the device is awake and in a known state.
     */
    void mpu6050_init() {
        // This will use I2C0 on pins 21 (SDA) and 20 (SCL)
        i2c_init(i2c_default, 400 * 1000);
        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA_PIN);
        gpio_pull_up(I2C_SCL_PIN);

        // Make the I2C pins available to picotool
        bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

        mpu6050_reset();
    }

    /**
     * mpu6050_reset
     *
     * Perform a basic software reset and wake-up sequence on the MPU6050.
     * Writes to register 0x6B (PWR_MGMT_1) — first to reset, then to clear
     * sleep bit. This is a minimal initialization; further configuration
     * (sample rates, ranges, filters) can be added as needed.
     */
    static void mpu6050_reset() {
        // Two byte reset. First byte register, second byte data
        // There are a load more options to set up the device in different ways that could be added here
        uint8_t buf[] = {0x6B, 0x80};
        i2c_write_blocking(i2c_default, ADDR, buf, 2, false);
        sleep_ms(100); // Allow device to reset and stabilize

        // Clear sleep mode (0x6B register, 0x00 value)
        buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
        i2c_write_blocking(i2c_default, ADDR, buf, 2, false); 
        sleep_ms(10); // Allow stabilization after waking up
    }

    /**
     * mpu6050_read_acceleration
     *
     * Read raw accelerometer registers (6 bytes starting at 0x3B) and
     * return them in a `vector_3d` structure. Values are raw 16-bit signed
     * integers combined from high/low bytes (big-endian), commonly used as
     * LSB counts. To convert to g use the ACCEL scale (e.g. `ACCEL_SENS_2G`).
     *
     * @return vector_3d with raw `.x`, `.y`, `.z` acceleration LSB values
     */
    vector_3d mpu6050_read_acceleration() {
        // For this particular device, we send the device the register we want to read
        // first, then subsequently read from the device. The register is auto incrementing
        // so we don't need to keep sending the register we want, just the first.

        uint8_t buffer[6];

        // Start reading acceleration registers from register 0x3B for 6 bytes
        uint8_t val = 0x3B;
        i2c_write_blocking(i2c_default, ADDR, &val, 1, true); // true to keep master control of bus
        i2c_read_blocking(i2c_default, ADDR, buffer, 6, false);

        vector_3d v = {
            .x = buffer[0] << 8 | buffer[1],
            .y = buffer[2] << 8 | buffer[3],
            .z = buffer[4] << 8 | buffer[5]
        };

        return v;
    }

    /**
     * mpu6050_read_gyro
     *
     * Read raw gyroscope registers (6 bytes starting at 0x43) and return
     * them in a `vector_3d`. Values are raw 16-bit signed integers (MSB/LSB
     * pairs). To convert to degrees/sec use the gyro sensitivity (e.g.
     * `GYRO_SENS_250DPS`).
     *
     * @return vector_3d with raw `.x`, `.y`, `.z` gyro LSB values
     */
    vector_3d mpu6050_read_gyro() {
        // For this particular device, we send the device the register we want to read
        // first, then subsequently read from the device. The register is auto incrementing
        // so we don't need to keep sending the register we want, just the first.

        uint8_t buffer[6];

        // Start gyro data from reg 0x43 for 6 bytes
        // The register is auto incrementing on each read
        uint8_t val = 0x43;
        i2c_write_blocking(i2c_default, ADDR, &val, 1, true);
        i2c_read_blocking(i2c_default, ADDR, buffer, 6, false);  // False - finished with bus

        vector_3d v = {
            .x = buffer[0] << 8 | buffer[1],
            .y = buffer[2] << 8 | buffer[3],
            .z = buffer[4] << 8 | buffer[5]
        };

        return v;
    }

#else
    #error "i2c_default not defined"
#endif
