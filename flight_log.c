#include "flight_log.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include "mpu6050.h"

#define CAPACITY 8800   // 17.6 s at 500 Hz; the motor test needs ~16.9 s. RAM-limited (28 B/sample).
#define HOST_STALL_TIMEOUT_US 5000000

// Fixed point: angles 0.01 deg, rates 0.1 deg/s, throttle and PID outputs 0.01 %, accel 0.001 g, vz 0.001 m/s,
// vbat 0.001 V.
typedef struct {
    int16_t roll, pitch, roll_rate, pitch_rate, yaw_rate;
    int16_t throttle, u_roll, u_pitch, u_yaw;
    int16_t ax, ay, az;
    int16_t vz;
    int16_t vbat;
} sample_t;

static sample_t samples[CAPACITY];
static unsigned count;

static int16_t quantize(float v, float scale) {
    float s = v * scale;
    if (s >  32767.0f) return  32767;
    if (s < -32767.0f) return -32767;
    return (int16_t)(s < 0.0f ? s - 0.5f : s + 0.5f);
}

void flight_log_record(float throttle, const attitude_t *att, float u_roll, float u_pitch, float u_yaw, float vz,
                       float vbat) {
    if (count >= CAPACITY) return;
    samples[count++] = (sample_t){
        .roll       = quantize(att->roll,       100.0f),
        .pitch      = quantize(att->pitch,      100.0f),
        .roll_rate  = quantize(att->roll_rate,   10.0f),
        .pitch_rate = quantize(att->pitch_rate,  10.0f),
        .yaw_rate   = quantize(att->yaw_rate,    10.0f),
        .throttle   = quantize(throttle,        100.0f),
        .u_roll     = quantize(u_roll,          100.0f),
        .u_pitch    = quantize(u_pitch,         100.0f),
        .u_yaw      = quantize(u_yaw,           100.0f),
        .ax         = quantize(att->accel_x,   1000.0f),
        .ay         = quantize(att->accel_y,   1000.0f),
        .az         = quantize(att->accel_z,   1000.0f),
        .vz         = quantize(vz,             1000.0f),
        .vbat       = quantize(vbat,           1000.0f),
    };
}

// stdio_usb silently drops output once the host hasn't read for 500 ms (garbled/missing rows),
// so only hand it what already fits in the CDC TX FIFO and wait for the host otherwise.
static bool usb_write(const char *s, int len) {
    while (len > 0) {
        absolute_time_t deadline = make_timeout_time_us(HOST_STALL_TIMEOUT_US);
        int avail;
        while ((avail = (int)tud_cdc_write_available()) == 0) {
            if (!stdio_usb_connected() || time_reached(deadline)) return false;
            sleep_us(100);
        }
        int n = len < avail ? len : avail;
        stdio_put_string(s, n, false, false);
        s += n;
        len -= n;
    }
    return true;
}

static bool usb_printf(const char *fmt, ...) {
    char line[160];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(line, sizeof line, fmt, args);
    va_end(args);
    if (len < 0) return false;
    if (len >= (int)sizeof line) len = sizeof line - 1;
    return usb_write(line, len);
}

void flight_log_dump(void) {
    if (!usb_printf("--- FLIGHT LOG BEGIN (%u samples, %lu i2c failures, %lu i2c recoveries) ---\n", count,
                    (unsigned long)mpu6050_i2c_failure_count(),
                    (unsigned long)mpu6050_bus_recovery_count())) return;
    if (!usb_printf("t_ms,throttle,roll,pitch,roll_rate,pitch_rate,yaw_rate,u_roll,u_pitch,u_yaw,ax,ay,az,vz,vbat\n")) return;
    for (unsigned i = 0; i < count; i++) {
        const sample_t *s = &samples[i];
        if (!usb_printf("%u,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        i * (ATTITUDE_SAMPLE_PERIOD_US / 1000),
                        s->throttle / 100.0f, s->roll / 100.0f, s->pitch / 100.0f,
                        s->roll_rate / 10.0f, s->pitch_rate / 10.0f, s->yaw_rate / 10.0f,
                        s->u_roll / 100.0f, s->u_pitch / 100.0f, s->u_yaw / 100.0f,
                        s->ax / 1000.0f, s->ay / 1000.0f, s->az / 1000.0f, s->vz / 1000.0f,
                        s->vbat / 1000.0f)) return;
    }
    usb_printf("--- FLIGHT LOG END ---\n");
}
