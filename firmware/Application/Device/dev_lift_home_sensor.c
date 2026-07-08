#include "dev_lift_home_sensor.h"

static uint8_t lift_home_sensor_read_level(lift_home_sensor_device_t *sensor)
{
    if ((sensor == 0) || (sensor->driver == 0) || (sensor->driver->get_level == 0)) {
        return 0U;
    }

    return sensor->driver->get_level(sensor->driver->ctx);
}

void lift_home_sensor_init(void *ctx)
{
    lift_home_sensor_device_t *sensor = (lift_home_sensor_device_t *)ctx;

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
        sensor->level = lift_home_sensor_read_level(sensor);
        sensor->last_level = sensor->level;
    }
}

void lift_home_sensor_update(void *ctx)
{
    lift_home_sensor_device_t *sensor = (lift_home_sensor_device_t *)ctx;
    uint8_t level;

    if ((sensor == 0) || (sensor->is_initialized == 0U)) {
        return;
    }

    level = lift_home_sensor_read_level(sensor);

    sensor->last_level = sensor->level;
    sensor->level = level;
    sensor->rising_edge = ((sensor->last_level == 0U) && (sensor->level != 0U)) ? 1U : 0U;
    sensor->falling_edge = ((sensor->last_level != 0U) && (sensor->level == 0U)) ? 1U : 0U;
}

uint8_t lift_home_sensor_get_level(void *ctx)
{
    lift_home_sensor_device_t *sensor = (lift_home_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->level : 0U;
}

uint8_t lift_home_sensor_get_last_level(void *ctx)
{
    lift_home_sensor_device_t *sensor = (lift_home_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->last_level : 0U;
}

uint8_t lift_home_sensor_get_rising_edge(void *ctx)
{
    lift_home_sensor_device_t *sensor = (lift_home_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->rising_edge : 0U;
}

uint8_t lift_home_sensor_get_falling_edge(void *ctx)
{
    lift_home_sensor_device_t *sensor = (lift_home_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->falling_edge : 0U;
}

uint8_t lift_home_sensor_is_initialized(void *ctx)
{
    lift_home_sensor_device_t *sensor = (lift_home_sensor_device_t *)ctx;

    return (sensor != 0) ? sensor->is_initialized : 0U;
}
