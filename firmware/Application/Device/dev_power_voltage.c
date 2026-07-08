#include "dev_power_voltage.h"

static uint16_t power_voltage_scale_raw(uint16_t raw, uint16_t ref_mv,
                                        uint16_t max_raw)
{
    if (max_raw == 0U) {
        return 0U;
    }

    return (uint16_t)(((uint32_t)raw * (uint32_t)ref_mv +
                       ((uint32_t)max_raw / 2UL)) /
                      (uint32_t)max_raw);
}

static uint16_t power_voltage_calc_power_mv(uint16_t adc_pin_mv,
                                            uint16_t voltage_scale_num,
                                            uint16_t voltage_scale_den)
{
    uint32_t power_mv;

    if (voltage_scale_den == 0U) {
        return 0U;
    }

    power_mv = ((uint32_t)adc_pin_mv * (uint32_t)voltage_scale_num +
                ((uint32_t)voltage_scale_den / 2UL)) /
               (uint32_t)voltage_scale_den;

    return (power_mv > 0xFFFFUL) ? 0xFFFFU : (uint16_t)power_mv;
}

static uint16_t power_voltage_calibrate_power_mv(uint16_t power_mv,
                                                 uint16_t cal_num,
                                                 uint16_t cal_den)
{
    uint32_t calibrated_mv;

    if ((cal_num == 0U) || (cal_den == 0U)) {
        return power_mv;
    }

    calibrated_mv = ((uint32_t)power_mv * (uint32_t)cal_num +
                     ((uint32_t)cal_den / 2UL)) /
                    (uint32_t)cal_den;

    return (calibrated_mv > 0xFFFFUL) ? 0xFFFFU : (uint16_t)calibrated_mv;
}

static uint16_t power_voltage_filter(power_voltage_device_t *voltage,
                                     uint16_t sample_mv)
{
    int32_t diff;
    uint32_t delta;
    uint32_t filtered_mv;

    if ((voltage->filter_shift == 0U) ||
        (voltage->filter_initialized == 0U)) {
        voltage->filter_initialized = 1U;
        return sample_mv;
    }

    filtered_mv = voltage->power_mv;
    diff = (int32_t)sample_mv - (int32_t)filtered_mv;
    if (diff >= 0) {
        delta = (uint32_t)diff >> voltage->filter_shift;
        if ((delta == 0U) && (diff != 0)) {
            delta = 1U;
        }
        filtered_mv += delta;
    } else {
        delta = (uint32_t)(-diff) >> voltage->filter_shift;
        if ((delta == 0U) && (diff != 0)) {
            delta = 1U;
        }
        filtered_mv -= delta;
    }

    return (filtered_mv > 0xFFFFUL) ? 0xFFFFU : (uint16_t)filtered_mv;
}

static uint8_t power_voltage_read_filtered_raw(power_voltage_device_t *voltage,
                                               uint16_t *raw)
{
    uint8_t sample_count;
    uint8_t i;
    uint16_t sample_raw = 0U;
    uint16_t min_raw = 0U;
    uint16_t max_raw = 0U;
    uint32_t raw_sum = 0UL;

    if ((voltage == 0) || (raw == 0) || (voltage->driver == 0) ||
        (voltage->driver->read_raw == 0)) {
        return 0U;
    }

    sample_count = voltage->adc_sample_count;
    if (sample_count == 0U) {
        sample_count = 1U;
    }

    for (i = 0U; i < sample_count; i++) {
        if (voltage->driver->read_raw(voltage->driver->ctx, &sample_raw) ==
            0U) {
            return 0U;
        }

        if (i == 0U) {
            min_raw = sample_raw;
            max_raw = sample_raw;
        } else {
            if (sample_raw < min_raw) {
                min_raw = sample_raw;
            }
            if (sample_raw > max_raw) {
                max_raw = sample_raw;
            }
        }
        raw_sum += sample_raw;
    }

    if (sample_count > 2U) {
        raw_sum -= min_raw;
        raw_sum -= max_raw;
        sample_count -= 2U;
    }

    *raw = (uint16_t)((raw_sum + ((uint32_t)sample_count / 2UL)) /
                      (uint32_t)sample_count);
    return 1U;
}

void power_voltage_init(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;

    if (voltage == 0) {
        return;
    }

    voltage->is_initialized = 0U;
    voltage->is_valid = 0U;
    voltage->adc_raw = 0U;
    voltage->adc_pin_mv = 0U;
    voltage->sample_power_mv = 0U;
    voltage->power_mv = 0U;
    voltage->filter_initialized = 0U;

    if ((voltage->driver != 0) && (voltage->driver->init != 0)) {
        voltage->is_initialized = voltage->driver->init(voltage->driver->ctx);
    }
}

void power_voltage_update(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;
    uint16_t raw = 0U;

    if ((voltage == 0) || (voltage->is_initialized == 0U) ||
        (voltage->driver == 0) || (voltage->driver->read_raw == 0)) {
        return;
    }

    if (power_voltage_read_filtered_raw(voltage, &raw) == 0U) {
        voltage->is_valid = 0U;
        return;
    }

    voltage->adc_raw = raw;
    voltage->adc_pin_mv = power_voltage_scale_raw(raw, voltage->adc_ref_mv,
                                                  voltage->adc_max_raw);

    voltage->sample_power_mv =
        power_voltage_calc_power_mv(voltage->adc_pin_mv,
                                    voltage->voltage_scale_num,
                                    voltage->voltage_scale_den);
    voltage->sample_power_mv =
        power_voltage_calibrate_power_mv(voltage->sample_power_mv,
                                         voltage->voltage_cal_num,
                                         voltage->voltage_cal_den);
    voltage->power_mv = power_voltage_filter(voltage, voltage->sample_power_mv);
    voltage->is_valid = 1U;
}

uint8_t power_voltage_is_initialized(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;

    return (voltage != 0) ? voltage->is_initialized : 0U;
}

uint8_t power_voltage_is_valid(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;

    return (voltage != 0) ? voltage->is_valid : 0U;
}

uint16_t power_voltage_get_adc_raw(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;

    return (voltage != 0) ? voltage->adc_raw : 0U;
}

uint16_t power_voltage_get_adc_pin_mv(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;

    return (voltage != 0) ? voltage->adc_pin_mv : 0U;
}

uint16_t power_voltage_get_sample_power_mv(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;

    return (voltage != 0) ? voltage->sample_power_mv : 0U;
}

uint16_t power_voltage_get_power_mv(void *ctx)
{
    power_voltage_device_t *voltage = (power_voltage_device_t *)ctx;

    return (voltage != 0) ? voltage->power_mv : 0U;
}
