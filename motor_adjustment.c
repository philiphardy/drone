#include <stdio.h>
#include "pico/mem_ops.h"
#include "pico/multicore.h"

static void task_entry();

void motor_adjustment_init() {
    // start task on core 1
    multicore_launch_core1(task_entry);
}

static void task_entry() {
    while (true) {
        uint32_t msg = multicore_fifo_pop_blocking();
        float pitch = *(float *)&msg;
        msg = multicore_fifo_pop_blocking();
        float roll = *(float *)&msg;
        printf("Pitch: %.2f  Roll: %.2f\n", pitch, roll);
    }
}
