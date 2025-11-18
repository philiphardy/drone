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

void flight_control_init() {
    mpu6050_init();
    motor_adjustment_init();
    task_entry();
}

static inline float radians_to_degrees(float r) {
    return r * (180.0f / M_PI);
}

// Call this every loop with dt in SECONDS
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

static inline float get_pitch() {
    return pitch - pitch_offset;
}

static inline float get_roll() {
    return roll - roll_offset;
}

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

static void notify_motor_adjustment() {
    // DO NOT REORDER!!!
    // The motor adjustment task relies on the queue being structured this way
    float pitch = get_pitch();
    multicore_fifo_push_blocking(*(uint32_t *)&pitch);
    float roll = get_roll();
    multicore_fifo_push_blocking(*(uint32_t *)&roll);
}