#include "dev_gas_sensor.h"

static uint8_t gas_sensor_read_level(gas_sensor_device_t *sensor)
{
    if ((sensor == 0) || (sensor->driver == 0) || (sensor->driver->get_level == 0)) {
        return 0U;
    }

    return sensor->driver->get_level(sensor->driver->ctx);
}

void gas_sensor_init(void *ctx)
{
    gas_sensor_device_t *sensor = (gas_sensor_device_t *)ctx;

    if (sensor == 0) {
        return;
    }

    sensor->is_initialized = 0U;
    sensor->level = 0U;
    sensor->last_level = 0U;
    sensor->rising_edge = 0U;
    sensor->falling_edge = 0U;

    if ((sensor->driver != 0) && (sensor->driver->init != 0)) {
        sensor->is_initialized = sensor->driver->init(sensor->driver->ctx);
    }

    if (sensor->is_initialized != 0U) {
        sensor->level = gas_sensor_read_level(sensor);
        sensor->last_level = sensor->level;
    }
}

void gas_sensor_update(void *ctx)
{
    gas_sensor_device_t *sensor = (gas_sensor_device_t *)ctx;
    uint8_t level;

    if ((sensor == 0) || (sensor->is_initialized == 0U)) {
        return;
    }

    level = gas_sensor_read_level(sensor);

    sensor->last_level = sensor->level;
    sensor->level = level;
    sensor->rising_edge = ((sensor->last_level == 0U) && (sensor->level != 0U)) ? 1U : 0U;
    sensor->falling_edge = ((sensor->last_level != 0U) && (sensor->level == 0U)) ? 1U : 0U;
}

uint8_t gas_sensor_get_level(void *ctx)
{
    gas_sensor_device_t *sensor = (gas_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->level : 0U;
}

uint8_t gas_sensor_get_last_level(void *ctx)
{
    gas_sensor_device_t *sensor = (gas_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->last_level : 0U;
}

uint8_t gas_sensor_get_rising_edge(void *ctx)
{
    gas_sensor_device_t *sensor = (gas_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->rising_edge : 0U;
}

uint8_t gas_sensor_get_falling_edge(void *ctx)
{
    gas_sensor_device_t *sensor = (gas_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->falling_edge : 0U;
}

uint8_t gas_sensor_is_initialized(void *ctx)
{
    gas_sensor_device_t *sensor = (gas_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->is_initialized : 0U;
}
