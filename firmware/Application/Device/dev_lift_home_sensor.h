#ifndef _DEV_LIFT_HOME_SENSOR_H_
#define _DEV_LIFT_HOME_SENSOR_H_

#include "drv_lift_home_sensor.h"
#include <stdint.h>

typedef struct {
    uint8_t is_initialized;
    uint8_t level;
    uint8_t last_level;
    uint8_t rising_edge;
    uint8_t falling_edge;

    const lift_home_sensor_driver_t *driver;
} lift_home_sensor_device_t;

void lift_home_sensor_init(void *ctx);
void lift_home_sensor_update(void *ctx);
uint8_t lift_home_sensor_get_level(void *ctx);
uint8_t lift_home_sensor_get_last_level(void *ctx);
uint8_t lift_home_sensor_get_rising_edge(void *ctx);
uint8_t lift_home_sensor_get_falling_edge(void *ctx);
uint8_t lift_home_sensor_is_initialized(void *ctx);

#endif /* _DEV_LIFT_HOME_SENSOR_H_ */
