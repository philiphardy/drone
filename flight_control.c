#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "mpu6050.h"
#include "motor_adjustment.h"

#define MICROSEC_PER_SEC 1e6f

#define ACCEL_SENS_2G   16384.0f     // LSB per g
#define GYRO_SENS_250DPS 131.0f      // LSB per deg/s

// Complementary filter constant
#define ALPHA 0.98f   // 98% gyro, 2% accel

#define LOOP_SLEEP_MS 5

#define CALIBRATION_CYCLES 250

typedef enum {
    CALIBRATING,
    OPERATING
} state_t;

static void task_entry();
static inline float radians_to_degrees(float r);
static void calibrate_starting_offsets();
static void complementary_filter_update(vector_3d * accel, vector_3d * gyro, float dt);
static void notify_motor_adjustment();
static inline float get_pitch();
static inline float get_roll();

static state_t state = CALIBRATING;

// Current fused angles (degrees)
static float pitch = 0.0f;
static float roll  = 0.0f;
static float pitch_offset = 0.0f;
static float roll_offset  = 0.0f;

/**
 * flight_control_init
 *
 * Initialize subsystems required for flight control:
 * - Initialize MPU6050 IMU via `mpu6050_init()`
 * - Initialize motor adjustment task via `motor_adjustment_init()`
 * - Enter the flight control task loop (`task_entry`).
 *
 * This function configures hardware and then transfers control to the
 * internal task loop; it does not return until the system halts.
 */
void flight_control_init() {
    mpu6050_init();
    motor_adjustment_init();
    task_entry();
}

/**
 * radians_to_degrees
 *
 * Convert radians to degrees.
 *
 * @param r Angle in radians
 * @return Angle in degrees
 */
static inline float radians_to_degrees(float r) {
    return r * (180.0f / M_PI);
}

/**
 * complementary_filter_update
 *
 * Update the fused pitch and roll estimates using a complementary filter.
 *
 * Inputs:
 * - `accel`: accelerometer readings (raw LSB)
 * - `gyro` : gyroscope readings (raw LSB)
 * - `dt`   : elapsed time in seconds since last update
 *
 * Behavior:
 * - Computes accelerometer-based pitch/roll (degrees) using `atan2f`.
 * - Integrates gyro rates (converted to deg/s) to predict change.
 * - Blends the gyro integration and accel estimate using the ALPHA constant.
 *
 * Side effects:
 * - Updates the module-wide `pitch` and `roll` (degrees).
 */
static void complementary_filter_update(vector_3d * accel, vector_3d * gyro, float dt)
{
    // Convert accel to g units
    float ax = accel->x / ACCEL_SENS_2G;
    float ay = accel->y / ACCEL_SENS_2G;
    float az = accel->z / ACCEL_SENS_2G;

    // --- Accelerometer angles ---
    // Pitch = rotation around Y axis
    float pitch_acc = radians_to_degrees(atan2f(ax, az));

    // Roll = rotation around X axis
    float roll_acc  = radians_to_degrees(atan2f(ay, az));

    // Convert gyro to deg/sec
    float gx = gyro->x / GYRO_SENS_250DPS;
    float gy = gyro->y / GYRO_SENS_250DPS;

    // --- Gyro integration (predict) ---
    pitch += gy * dt;     // gy rotates pitch
    roll  += gx * dt;     // gx rotates roll

    // --- Complementary filter blend ---
    pitch = ALPHA * pitch + (1.0f - ALPHA) * pitch_acc;
    roll  = ALPHA * roll  + (1.0f - ALPHA) * roll_acc;
}

/**
 * get_pitch
 *
 * Return the current fused pitch (degrees) with calibration offset removed.
 */
static inline float get_pitch() {
    return pitch - pitch_offset;
}

/**
 * get_roll
 *
 * Return the current fused roll (degrees) with calibration offset removed.
 */
static inline float get_roll() {
    return roll - roll_offset;
}

/**
 * task_entry
 *
 * Main flight control loop. Responsibilities:
 * - Read accelerometer and gyro samples from the IMU
 * - Compute elapsed time `dt` using `time_us_32()` (converted to seconds)
 * - Run the complementary filter to update fused angles
 * - Perform a short calibration on startup (see `CALIBRATION_CYCLES`)
 * - When operating, send pitch and roll to the motor adjustment core
 *
 * The loop sleeps for `LOOP_SLEEP_MS` milliseconds each iteration.
 */
static void task_entry() {
    uint32_t last_time = 0;
    uint8_t cycles = 0;

    while (true) {
        vector_3d acceleration = mpu6050_read_acceleration();
        vector_3d gyro = mpu6050_read_gyro();

        uint32_t now = time_us_32();
        float dt = (now - last_time) / MICROSEC_PER_SEC; // Convert to seconds
        last_time = now;

        complementary_filter_update(&acceleration, &gyro, dt);

        if (CALIBRATING == state && cycles >= CALIBRATION_CYCLES) {
            state = OPERATING;
            pitch_offset = pitch;
            roll_offset = roll;
        } else if (OPERATING == state) {
            // Pass pitch and roll to the motor adjustment task
            notify_motor_adjustment();
        }

        sleep_ms(LOOP_SLEEP_MS);
        cycles++;
    }
}

/**
 * notify_motor_adjustment
 *
 * Package the current pitch and roll values into the multicore FIFO so the
 * motor adjustment task (running on core1) can consume them. The order is
 * critical: pitch is pushed first, then roll. Values are sent as their raw
 * 32-bit bit-pattern (`float` -> `uint32_t`) so the receiving core can
 * reinterpret the bits back into floats.
 */
static void notify_motor_adjustment() {
    // DO NOT REORDER!!!
    // The motor adjustment task relies on the queue being structured this way
    float pitch = get_pitch();
    multicore_fifo_push_blocking(*(uint32_t *)&pitch);
    float roll = get_roll();
    multicore_fifo_push_blocking(*(uint32_t *)&roll);
}