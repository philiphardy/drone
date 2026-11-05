#include "motor_test.h"

#include <math.h>
#include "pico/stdlib.h"
#include "attitude.h"
#include "failsafe.h"
#include "flight_log.h"
#include "motor.h"

#define SPINUP_MS    1500   // first settle from stopped; ESCs need ~0.4 s before producing torque
#define SETTLE_MS    1000
#define BOOST_MS      600
#define RECOVER_MS    600
#define BOOST_PERCENT   4.0f
#define MAX_TILT_DEG   45.0f

// Kept below hover (~26%) so the untethered drone stays on the floor.
static const float BASELINES[] = { 14.0f, 18.0f };
// Single motors (GP0..GP3), then GP0+GP3 and GP1+GP2, which spin opposite ways.
static const unsigned MASKS[] = { 0x1, 0x2, 0x4, 0x8, 0x9, 0x6 };

// Logs throttle = baseline, u_roll = boosted-motor mask (0 = baseline only), u_pitch = boost percent.
static void run_step(float baseline, unsigned mask, unsigned duration_ms) {
    for (unsigned m = 0; m < MOTOR_COUNT; m++) {
        motor_set_motor_throttle_percent(m, baseline + ((mask >> m) & 1u ? BOOST_PERCENT : 0.0f));
    }

    const float dt = ATTITUDE_SAMPLE_PERIOD_US * 1e-6f;
    const float boost = mask ? BOOST_PERCENT : 0.0f;
    absolute_time_t next_tick = get_absolute_time();
    for (unsigned i = 0; i < duration_ms * 1000u / ATTITUDE_SAMPLE_PERIOD_US; i++) {
        next_tick = delayed_by_us(next_tick, ATTITUDE_SAMPLE_PERIOD_US);
        sleep_until(next_tick);

        attitude_t att;
        if (!attitude_update(dt, &att)) continue;
        if (fabsf(att.roll) > MAX_TILT_DEG || fabsf(att.pitch) > MAX_TILT_DEG) {
            failsafe("tilt limit exceeded");
        }
        flight_log_record(baseline, &att, (float)mask, boost, 0.0f);
    }
}

void motor_test_run(void) {
    for (unsigned b = 0; b < sizeof BASELINES / sizeof BASELINES[0]; b++) {
        run_step(BASELINES[b], 0, b == 0 ? SPINUP_MS : SETTLE_MS);
        for (unsigned s = 0; s < sizeof MASKS / sizeof MASKS[0]; s++) {
            run_step(BASELINES[b], MASKS[s], BOOST_MS);
            run_step(BASELINES[b], 0, RECOVER_MS);
        }
    }
    failsafe("motor test complete");
}
