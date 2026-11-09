#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

// 2S pack sensed on GP28 (ADC2) through a 20k/10k divider (divide by 3).
void battery_init(void);

// Raw 12-bit ADC count averaged over `samples` conversions (~2 us each).
uint16_t battery_read_raw(unsigned samples);

float battery_raw_to_volts(uint16_t raw);

float battery_read_volts(unsigned samples);

#endif // BATTERY_H
