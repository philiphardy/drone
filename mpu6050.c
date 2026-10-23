#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

// MPU6050 I2C address (AD0 tied low -> 0x68; tie AD0 high for 0x69)
#define ADDR 0x68

// I2C pin assignment — GPIO numbers, not physical pin numbers.
// On a Pico 1 (2020), GP26 = physical pin 31 and GP27 = physical pin 32.
// GP26/GP27 map to the i2c1 peripheral (NOT i2c0, which is GP20/GP21).
#define I2C_SDA_PIN 26   // GP26  (physical pin 31 on Pico 1)
#define I2C_SCL_PIN 27   // GP27  (physical pin 32 on Pico 1)

// The peripheral instance to drive. This must match the pin mapping above.
#define MPU_I2C i2c1

// ── Logging ──────────────────────────────────────────────────────
// MPU_ERR is always on; it's used only by init/reset, which run once.
// MPU_DBG is off by default and used in the hot path (read_* and the
// low-level burst read) — enable it temporarily to diagnose I2C
// failures inside the control loop. NOTE: printf via USB CDC is slow
// (hundreds of µs per call). If the bus is persistently failing you
// will noticeably slow the loop with this on; use it for debugging,
// then set it back to 0.
#ifndef MPU6050_DEBUG
#define MPU6050_DEBUG 1
#endif

#define MPU_ERR(fmt, ...) printf("mpu6050 ERROR: " fmt, ##__VA_ARGS__)

#if MPU6050_DEBUG
#define MPU_DBG(fmt, ...) printf("mpu6050: " fmt, ##__VA_ARGS__)
#else
#define MPU_DBG(fmt, ...) ((void)0)
#endif

// Assemble two big-endian bytes into a signed 16-bit value.
// The cast to int16_t makes the narrowing explicit (implementation-defined
// in C for out-of-range unsigned->signed conversion, well-defined on GCC
// / ARM and matches how the MPU6050 encodes negative readings).
// WARNING: do not pass arguments with side effects (e.g. buf[i++]) —
// arguments are evaluated more than once conceptually inside the macro.
#define I16(highByte, lowByte) \
    ((int16_t)((((uint16_t)(highByte)) << 8) | (uint8_t)(lowByte)))

// Register addresses (RM-MPU-6000A register map)
enum {
    REG_SMPLRT_DIV    = 0x19,
    REG_CONFIG        = 0x1A,
    REG_GYRO_CONFIG   = 0x1B,
    REG_ACCEL_CONFIG  = 0x1C,
    REG_ACCEL_XOUT_H  = 0x3B,
    REG_GYRO_XOUT_H   = 0x43,
    REG_PWR_MGMT_1    = 0x6B,
    REG_WHO_AM_I      = 0x75,

    WHO_AM_I_EXPECTED = 0x68,
};

// Private scale factors — exposed only via the accessors below.
// Must be float, not double: RP2040 has no hardware double unit and
// unsuffixed literals would silently fall back to software emulation.
static const float ACCEL_SCALE = 16384.0f;   // LSB per g
static const float GYRO_SCALE  = 131.0f;     // LSB per °/s

static bool mpu6050_reset(void);

// Write one register. Returns false on I2C failure and logs the reason.
static bool mpu6050_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int n = i2c_write_blocking(MPU_I2C, ADDR, buf, 2, false);
    if (n != 2) {
        MPU_ERR("i2c write reg=0x%02X val=0x%02X failed (ret=%d)\n",
                reg, val, n);
        return false;
    }
    return true;
}

// Burst-read `count` bytes starting at `reg` into `dest`.
// Returns true on success, false on any I2C failure (dest left untouched).
// Logs under MPU_DBG only, because this is called from both init and the
// hot loop — we don't want to spam stdout on every failed control cycle.
static bool mpu6050_burst_read(uint8_t reg, uint8_t *dest, size_t count) {
    int nw = i2c_write_blocking(MPU_I2C, ADDR, &reg, 1, true);
    if (nw != 1) {
        MPU_DBG("burst_read reg=0x%02X: write phase failed (ret=%d)\n",
                reg, nw);
        return false;
    }
    int nr = i2c_read_blocking(MPU_I2C, ADDR, dest, count, false);
    if (nr != (int)count) {
        MPU_DBG("burst_read reg=0x%02X: read phase failed (got %d, want %zu)\n",
                reg, nr, count);
        return false;
    }
    return true;
}

    /**
     * mpu6050_init
     *
     * Initialize I2C1 at 400 kHz on GP26 (SDA) / GP27 (SCL), reset the
     * MPU6050, and apply the configuration the Kalman filter expects:
     *   - DLPF_CFG   = 3 (44 Hz accel / 42 Hz gyro bandwidth)
     *   - SMPLRT_DIV = 9 (100 Hz output rate with DLPF enabled)
     *   - ±250 °/s gyro (131 LSB/(°/s))
     *   - ±2 g accel    (16384 LSB/g)
     *
     * Returns true only if reset, WHO_AM_I read, and all four config
     * writes succeeded. On failure, individual reasons are printed via
     * MPU_ERR before returning.
     */
    bool mpu6050_init(void) {
        i2c_init(MPU_I2C, 400 * 1000);
        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA_PIN);
        gpio_pull_up(I2C_SCL_PIN);

        // Make the I2C pins available to picotool
        bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

        // Reset first: DEVICE_RESET wipes all config registers to defaults
        if (!mpu6050_reset()) {
            MPU_ERR("reset failed — aborting init\n");
            return false;
        }

        // Sanity check: WHO_AM_I should read 0x68 on a genuine MPU6050.
        // Some clones ship an MPU6500 (WHO_AM_I = 0x70); the driver still
        // works, so a mismatch is informational rather than fatal.
        uint8_t who = 0;
        if (!mpu6050_burst_read(REG_WHO_AM_I, &who, 1)) {
            MPU_ERR("WHO_AM_I read failed (no response at addr 0x%02X)\n",
                    ADDR);
            return false;
        }
        if (who != WHO_AM_I_EXPECTED) {
            printf("mpu6050: WHO_AM_I = 0x%02X (expected 0x%02X) "
                   "— continuing anyway\n", who, WHO_AM_I_EXPECTED);
        } else {
            printf("mpu6050: WHO_AM_I check passed (0x%02X)\n", who);
        }

        // Config writes — reported individually so you see all problems
        // in one boot instead of fixing them one at a time.
        static const struct {
            uint8_t reg;
            uint8_t val;
            const char *name;
        } cfg[] = {
            { REG_CONFIG,       0x03, "CONFIG (DLPF_CFG=3)"         },
            { REG_SMPLRT_DIV,   0x09, "SMPLRT_DIV (=9, 100 Hz)"     },
            { REG_GYRO_CONFIG,  0x00, "GYRO_CONFIG (FS_SEL=0, ±250)"},
            { REG_ACCEL_CONFIG, 0x00, "ACCEL_CONFIG (AFS_SEL=0, ±2g)"},
        };

        bool all_ok = true;
        for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
            if (!mpu6050_write_reg(cfg[i].reg, cfg[i].val)) {
                MPU_ERR("%s write failed\n", cfg[i].name);
                all_ok = false;
            }
        }
        if (!all_ok) {
            MPU_ERR("initialization incomplete — sensor may not behave "
                    "as expected\n");
            return false;
        }

        printf("mpu6050: init OK (i2c1, SDA=GP%d, SCL=GP%d)\n",
               I2C_SDA_PIN, I2C_SCL_PIN);
        return true;
    }

    /**
     * mpu6050_reset
     *
     * Assert DEVICE_RESET, wait for the device to come back, then clear SLEEP
     * and select the PLL with X-gyro reference as the clock source.
     * Returns false if either write fails.
     */
    static bool mpu6050_reset(void) {
        if (!mpu6050_write_reg(REG_PWR_MGMT_1, 0x80)) {
            MPU_ERR("DEVICE_RESET write failed\n");
            return false;
        }
        sleep_ms(100);                             // allow reset to settle

        if (!mpu6050_write_reg(REG_PWR_MGMT_1, 0x01)) {
            MPU_ERR("wake / PLL select write failed\n");
            return false;
        }
        sleep_ms(10);
        return true;
    }

    float mpu6050_accel_scale(void) {
        return ACCEL_SCALE;
    }

    float mpu6050_gyro_scale(void) {
        return GYRO_SCALE;
    }

    /**
     * mpu6050_read_acceleration
     *
     * Read the 6 accelerometer bytes starting at 0x3B into *out as signed
     * 16-bit LSB counts. Divide by mpu6050_accel_scale() for g.
     * Returns false on NULL output or I2C failure; *out is untouched.
     * Detailed reason goes to stdout only when MPU6050_DEBUG=1.
     */
    bool mpu6050_read_acceleration(vector_3d *out) {
        if (!out) {
            MPU_DBG("read_acceleration: NULL output pointer\n");
            return false;
        }

        uint8_t buf[6];
        if (!mpu6050_burst_read(REG_ACCEL_XOUT_H, buf, sizeof buf)) {
            MPU_DBG("read_acceleration: I2C failure\n");
            return false;
        }

        out->x = I16(buf[0], buf[1]);
        out->y = I16(buf[2], buf[3]);
        out->z = I16(buf[4], buf[5]);
        return true;
    }

    /**
     * mpu6050_read_gyro
     *
     * Read the 6 gyroscope bytes starting at 0x43 into *out as signed
     * 16-bit LSB counts. Divide by mpu6050_gyro_scale() for °/s.
     * Returns false on NULL output or I2C failure; *out is untouched.
     * Detailed reason goes to stdout only when MPU6050_DEBUG=1.
     */
    bool mpu6050_read_gyro(vector_3d *out) {
        if (!out) {
            MPU_DBG("read_gyro: NULL output pointer\n");
            return false;
        }

        uint8_t buf[6];
        if (!mpu6050_burst_read(REG_GYRO_XOUT_H, buf, sizeof buf)) {
            MPU_DBG("read_gyro: I2C failure\n");
            return false;
        }

        out->x = I16(buf[0], buf[1]);
        out->y = I16(buf[2], buf[3]);
        out->z = I16(buf[4], buf[5]);
        return true;
    }

    /**
     * mpu6050_read_sample
     *
     * Read accel + gyro in one 14-byte burst from 0x3B, so both vectors come
     * from the same internal sample. Layout:
     *   AXH AXL AYH AYL AZH AZL TH TL GXH GXL GYH GYL GZH GZL
     * (temperature bytes 6..7 are skipped.)
     * Returns false on NULL output or I2C failure; outputs untouched.
     * Detailed reason goes to stdout only when MPU6050_DEBUG=1.
     */
    bool mpu6050_read_sample(vector_3d *accel, vector_3d *gyro) {
        if (!accel || !gyro) {
            MPU_DBG("read_sample: NULL output pointer(s)\n");
            return false;
        }

        uint8_t buf[14];
        if (!mpu6050_burst_read(REG_ACCEL_XOUT_H, buf, sizeof buf)) {
            MPU_DBG("read_sample: I2C failure\n");
            return false;
        }

        accel->x = I16(buf[0],  buf[1]);
        accel->y = I16(buf[2],  buf[3]);
        accel->z = I16(buf[4],  buf[5]);
        // buf[6..7] = temperature
        gyro->x  = I16(buf[8],  buf[9]);
        gyro->y  = I16(buf[10], buf[11]);
        gyro->z  = I16(buf[12], buf[13]);
        return true;
    }