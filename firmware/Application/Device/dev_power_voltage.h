#ifndef _DEV_POWER_VOLTAGE_H_
#define _DEV_POWER_VOLTAGE_H_

#include "drv_power_voltage.h"
#include <stdint.h>

typedef struct {
    uint8_t is_initialized;
    uint8_t is_valid;
    uint16_t adc_raw;
    uint16_t adc_pin_mv;
    uint16_t sample_power_mv;
    uint16_t power_mv;

    uint16_t adc_ref_mv;
    uint16_t adc_max_raw;
    uint16_t voltage_scale_num;
    uint16_t voltage_scale_den;
    uint16_t voltage_cal_num;
    uint16_t voltage_cal_den;
    uint8_t adc_sample_count;
    uint8_t filter_shift;
    uint8_t filter_initialized;
    const power_voltage_driver_t *driver;
} power_voltage_device_t;

void power_voltage_init(void *ctx);
void power_voltage_update(void *ctx);
uint8_t power_voltage_is_initialized(void *ctx);
uint8_t power_voltage_is_valid(void *ctx);
uint16_t power_voltage_get_adc_raw(void *ctx);
uint16_t power_voltage_get_adc_pin_mv(void *ctx);
uint16_t power_voltage_get_sample_power_mv(void *ctx);
uint16_t power_voltage_get_power_mv(void *ctx);

#endif /* _DEV_POWER_VOLTAGE_H_ */
