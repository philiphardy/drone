#include "battery_test.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "battery.h"

#define PRINT_PERIOD_MS 500
#define CELLS           2
#define AVG_SAMPLES     64

void battery_test_run(void) {
    while (true) {
        uint16_t raw = battery_read_raw(AVG_SAMPLES);
        float v = battery_raw_to_volts(raw);
        printf("VBAT %.3f V  (%.3f V/cell, raw %u)\n", v, v / CELLS, raw);
        sleep_ms(PRINT_PERIOD_MS);
    }
}
