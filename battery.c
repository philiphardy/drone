#include "battery.h"

#include "hardware/adc.h"

#define BATTERY_GPIO          28
#define BATTERY_ADC_INPUT     2      // GP28 = ADC2
#define ADC_VREF_VOLTS        3.3f
#define ADC_FULL_SCALE        4096.0f
#define DIVIDER_RATIO         3.0f   // (20k + 10k) / 10k

void battery_init(void) {
    adc_init();
    adc_gpio_init(BATTERY_GPIO);
}

uint16_t battery_read_raw(unsigned samples) {
    adc_select_input(BATTERY_ADC_INPUT);
    uint32_t sum = 0;
    for (unsigned i = 0; i < samples; i++) {
        sum += adc_read();
    }
    return (uint16_t)(sum / samples);
}

float battery_raw_to_volts(uint16_t raw) {
    return raw * (ADC_VREF_VOLTS / ADC_FULL_SCALE) * DIVIDER_RATIO;
}

float battery_read_volts(unsigned samples) {
    return battery_raw_to_volts(battery_read_raw(samples));
}
