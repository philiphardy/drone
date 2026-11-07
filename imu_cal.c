#include "imu_cal.h"

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "attitude.h"

#define CAL_SAMPLES      500    // 5 s at 100 Hz
#define CAL_PERIOD_US    10000
#define MAX_STILL_SD_DEG 0.5f   // a wider spread means the drone moved

void imu_cal_run(void) {
    while (true) {
        double sum_r = 0, sum_p = 0, sq_r = 0, sq_p = 0;
        unsigned n = 0;
        for (unsigned i = 0; i < CAL_SAMPLES; i++) {
            float r, p;
            if (attitude_read_imu_angles(&r, &p)) {
                sum_r += r;  sq_r += (double)r * r;
                sum_p += p;  sq_p += (double)p * p;
                n++;
            }
            sleep_us(CAL_PERIOD_US);
        }

        double roll = sum_r / n, pitch = sum_p / n;
        double sd_r = sqrt(fmax(sq_r / n - roll * roll, 0.0));
        double sd_p = sqrt(fmax(sq_p / n - pitch * pitch, 0.0));
        bool moved = sd_r > MAX_STILL_SD_DEG || sd_p > MAX_STILL_SD_DEG;
        printf("IMU_ROLL_OFFSET_DEG %+.2f  IMU_PITCH_OFFSET_DEG %+.2f  (sd %.2f/%.2f deg, %u samples)%s\n",
               roll, pitch, sd_r, sd_p, n, moved ? "  moved - discard" : "");
    }
}
