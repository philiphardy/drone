#include "flight_log.h"

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"

#define CAPACITY 9000   // 18 s at 500 Hz; the motor test needs ~17 s

// Fixed point: angles 0.01 deg, rates 0.1 deg/s, throttle and PID outputs 0.01 %, accel 0.001 g, vz 0.001 m/s.
typedef struct {
    int16_t roll, pitch, roll_rate, pitch_rate, yaw_rate;
    int16_t throttle, u_roll, u_pitch, u_yaw;
    int16_t ax, ay, az;
    int16_t vz;
} sample_t;

static sample_t samples[CAPACITY];
static unsigned count;

static int16_t quantize(float v, float scale) {
    float s = v * scale;
    if (s >  32767.0f) return  32767;
    if (s < -32767.0f) return -32767;
    return (int16_t)(s < 0.0f ? s - 0.5f : s + 0.5f);
}

void flight_log_record(float throttle, const attitude_t *att, float u_roll, float u_pitch, float u_yaw, float vz) {
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
    };
}

void flight_log_dump(void) {
    printf("--- FLIGHT LOG BEGIN (%u samples) ---\n", count);
    printf("t_ms,throttle,roll,pitch,roll_rate,pitch_rate,yaw_rate,u_roll,u_pitch,u_yaw,ax,ay,az,vz\n");
    for (unsigned i = 0; i < count; i++) {
        const sample_t *s = &samples[i];
        printf("%u,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\n",
               i * (ATTITUDE_SAMPLE_PERIOD_US / 1000),
               s->throttle / 100.0f, s->roll / 100.0f, s->pitch / 100.0f,
               s->roll_rate / 10.0f, s->pitch_rate / 10.0f, s->yaw_rate / 10.0f,
               s->u_roll / 100.0f, s->u_pitch / 100.0f, s->u_yaw / 100.0f,
               s->ax / 1000.0f, s->ay / 1000.0f, s->az / 1000.0f, s->vz / 1000.0f);
    }
    printf("--- FLIGHT LOG END ---\n");
}
