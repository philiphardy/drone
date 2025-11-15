#include <stdio.h>
#include "pico/multicore.h"
#include "vector_3d.h"

static void motor_adjustment_task_entry();

void motor_adjustment_init() {
    // start task on core 1
    multicore_launch_core1(motor_adjustment_task_entry);
}

static void motor_adjustment_task_entry() {
    while (true) {
        vector_3d currAccel = (vector_3d) {
            .x = (int16_t)multicore_fifo_pop_blocking(),
            .y = (int16_t)multicore_fifo_pop_blocking(),
            .z = (int16_t)multicore_fifo_pop_blocking()
        };
        printv("MA current accel", &currAccel);

        vector_3d deltaAccel = (vector_3d) {
            .x = (int16_t)multicore_fifo_pop_blocking(),
            .y = (int16_t)multicore_fifo_pop_blocking(),
            .z = (int16_t)multicore_fifo_pop_blocking()
        };
        printv("MA delta accel", &deltaAccel);
    }
}
