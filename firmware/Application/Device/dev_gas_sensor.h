#ifndef _DEV_GAS_SENSOR_H_
#define _DEV_GAS_SENSOR_H_

#include "drv_gas_sensor.h"
#include <stdint.h>

typedef struct {
    uint8_t is_initialized;
    uint8_t level;
    uint8_t last_level;
    uint8_t rising_edge;
    uint8_t falling_edge;

    const gas_sensor_driver_t *driver;
} gas_sensor_device_t;

void gas_sensor_init(void *ctx);
void gas_sensor_update(void *ctx);
uint8_t gas_sensor_get_level(void *ctx);
uint8_t gas_sensor_get_last_level(void *ctx);
uint8_t gas_sensor_get_rising_edge(void *ctx);
uint8_t gas_sensor_get_falling_edge(void *ctx);
uint8_t gas_sensor_is_initialized(void *ctx);

#endif /* _DEV_GAS_SENSOR_H_ */
