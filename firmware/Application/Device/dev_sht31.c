#include "dev_sht31.h"

#define SHT31_RAW_MAX (65535UL)
#define SHT31_TEMPERATURE_OFFSET_CENTI_C (4500L)
#define SHT31_TEMPERATURE_SCALE_CENTI_C (17500UL)
#define SHT31_HUMIDITY_SCALE_CENTI_PCT (10000UL)

static int16_t sht31_convert_temperature(uint16_t raw)
{
    int32_t temperature;

    temperature = (int32_t)(((uint32_t)raw * SHT31_TEMPERATURE_SCALE_CENTI_C +
                             (SHT31_RAW_MAX / 2UL)) /
                            SHT31_RAW_MAX);
    temperature -= SHT31_TEMPERATURE_OFFSET_CENTI_C;

    return (int16_t)temperature;
}

static uint16_t sht31_convert_humidity(uint16_t raw)
{
    uint32_t humidity;

    humidity = ((uint32_t)raw * SHT31_HUMIDITY_SCALE_CENTI_PCT +
                (SHT31_RAW_MAX / 2UL)) /
               SHT31_RAW_MAX;

    return (humidity > SHT31_HUMIDITY_SCALE_CENTI_PCT)
               ? (uint16_t)SHT31_HUMIDITY_SCALE_CENTI_PCT
               : (uint16_t)humidity;
}

static void sht31_set_error(sht31_device_t *sensor)
{
    if (sensor == 0) {
        return;
    }

    if (sensor->error_count < 0xFFU) {
        sensor->error_count++;
    }

    if ((sensor->is_valid == 0U) ||
        ((sensor->error_limit != 0U) && (sensor->error_count >= sensor->error_limit))) {
        sensor->status = SHT31_STATUS_ERR;
    }
}

static void sht31_store_sample(sht31_device_t *sensor,
                               const sht31_raw_data_t *sample)
{
    if ((sensor == 0) || (sample == 0)) {
        return;
    }

    sensor->temperature_raw = sample->temperature_raw;
    sensor->humidity_raw = sample->humidity_raw;
    sensor->temperature_centi_c = sht31_convert_temperature(sample->temperature_raw);
    sensor->humidity_centi_pct = sht31_convert_humidity(sample->humidity_raw);
    sensor->is_valid = 1U;
    sensor->error_count = 0U;
    sensor->status = SHT31_STATUS_OK;
}

void sht31_init(void *ctx)
{
    sht31_device_t *sensor = (sht31_device_t *)ctx;

    if (sensor == 0) {
        return;
    }

    sensor->is_initialized = 0U;
    sensor->is_valid = 0U;
    sensor->error_count = 0U;
    sensor->status = SHT31_STATUS_ERR;
    sensor->measure_state = SHT31_MEASURE_IDLE;
    sensor->temperature_raw = 0U;
    sensor->humidity_raw = 0U;
    sensor->temperature_centi_c = 0;
    sensor->humidity_centi_pct = 0U;
    sensor->period_elapsed_ms = sensor->measure_period_ms;
    sensor->wait_elapsed_ms = 0U;

    if ((sensor->driver != 0) && (sensor->driver->init != 0)) {
        sensor->is_initialized = sensor->driver->init(sensor->driver->ctx);
    }

    if (sensor->is_initialized != 0U) {
        sensor->status = SHT31_STATUS_WAIT;
    }
}

void sht31_update(void *ctx, uint32_t dt_ms)
{
    sht31_device_t *sensor = (sht31_device_t *)ctx;
    sht31_raw_data_t sample;

    if ((sensor == 0) || (sensor->is_initialized == 0U) ||
        (sensor->driver == 0)) {
        return;
    }

    if (sensor->measure_state == SHT31_MEASURE_WAIT) {
        sensor->wait_elapsed_ms += dt_ms;
        if (sensor->wait_elapsed_ms < sensor->measure_wait_ms) {
            return;
        }

        sensor->measure_state = SHT31_MEASURE_IDLE;
        sensor->period_elapsed_ms = 0U;

        if ((sensor->driver->read_measurement != 0) &&
            (sensor->driver->read_measurement(sensor->driver->ctx, &sample) != 0U)) {
            sht31_store_sample(sensor, &sample);
        } else {
            sht31_set_error(sensor);
        }
        return;
    }

    sensor->period_elapsed_ms += dt_ms;
    if (sensor->period_elapsed_ms < sensor->measure_period_ms) {
        return;
    }

    if ((sensor->driver->start_measurement != 0) &&
        (sensor->driver->start_measurement(sensor->driver->ctx) != 0U)) {
        sensor->measure_state = SHT31_MEASURE_WAIT;
        sensor->wait_elapsed_ms = 0U;
        if (sensor->is_valid == 0U) {
            sensor->status = SHT31_STATUS_WAIT;
        }
    } else {
        sensor->period_elapsed_ms = 0U;
        sht31_set_error(sensor);
    }
}

uint8_t sht31_is_initialized(void *ctx)
{
    sht31_device_t *sensor = (sht31_device_t *)ctx;

    return (sensor != 0) ? sensor->is_initialized : 0U;
}

uint8_t sht31_is_valid(void *ctx)
{
    sht31_device_t *sensor = (sht31_device_t *)ctx;

    return (sensor != 0) ? sensor->is_valid : 0U;
}

uint8_t sht31_get_status(void *ctx)
{
    sht31_device_t *sensor = (sht31_device_t *)ctx;

    return (sensor != 0) ? (uint8_t)sensor->status : (uint8_t)SHT31_STATUS_ERR;
}

int16_t sht31_get_temperature_centi_c(void *ctx)
{
    sht31_device_t *sensor = (sht31_device_t *)ctx;

    return (sensor != 0) ? sensor->temperature_centi_c : 0;
}

uint16_t sht31_get_humidity_centi_pct(void *ctx)
{
    sht31_device_t *sensor = (sht31_device_t *)ctx;

    return (sensor != 0) ? sensor->humidity_centi_pct : 0U;
}
