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

static void flight_control_begin();
static void complementary_filter_update(vector_3d * accel, vector_3d * gyro, float dt);
static void notify_motor_adjustment();

// Current fused angles (degrees)
static float pitch = 0.0f;
static float roll  = 0.0f;

void flight_control_init() {
    mpu6050_init();
    motor_adjustment_init();
    // TODO: Calibrate to zero out starting roll and pitch
    flight_control_begin();
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

    // Convert gyro to deg/sec
    float gx = gyro->x / GYRO_SENS_250DPS;
    float gy = gyro->y / GYRO_SENS_250DPS;

    // --- Accelerometer angles ---
    // Pitch = rotation around Y axis
    float pitch_acc = radians_to_degrees(atan2f(ax, sqrtf(ay * ay + az * az)));
    // Roll = rotation around X axis
    float roll_acc  = radians_to_degrees(atan2f(ay, az));

    // --- Gyro integration (predict) ---
    pitch -= gx * dt;     // gx rotates pitch
    roll  += gy * dt;     // gy rotates roll

    // --- Complementary filter blend ---
    pitch = ALPHA * pitch + (1.0f - ALPHA) * pitch_acc;
    roll  = ALPHA * roll  + (1.0f - ALPHA) * roll_acc;
}

static void flight_control_begin() {
    uint32_t lastTime = 0;

    while (true) {
        vector_3d acceleration = mpu6050_read_acceleration();
        vector_3d gyro = mpu6050_read_gyro();

        uint32_t now = time_us_32();
        float dt = (now - lastTime) / MICROSEC_PER_SEC; // Convert to seconds
        lastTime = now;

        complementary_filter_update(&acceleration, &gyro, dt);

        notify_motor_adjustment();

        // Pass pitch and roll to the motor adjustment task
        sleep_ms(LOOP_SLEEP_MS);
    }
}

static void notify_motor_adjustment() {
    uint32_t p = *(uint32_t *)&pitch;
    uint32_t r = *(uint32_t *)&roll;
    // DO NOT REORDER!!!
    // The motor adjustment task relies on the queue being structured this way
    multicore_fifo_push_blocking(p);
    multicore_fifo_push_blocking(r);
}