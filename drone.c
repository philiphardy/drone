#include <stdio.h>
#include "pico/stdlib.h"
#include "flight_control.h"


/**
 * main
 *
 * Program entry point. Initializes stdio (USB/UART) and starts the
 * flight control subsystem. On the Raspberry Pi Pico the flight control
 * code launches tasks and the function does not explicitly return.
 */
int main()
{
    stdio_init_all();
    flight_control_init();
}
