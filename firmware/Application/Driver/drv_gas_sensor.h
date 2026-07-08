#ifndef _DRV_GAS_SENSOR_H_
#define _DRV_GAS_SENSOR_H_

#include <stdint.h>

typedef struct {
    const void *ctx;
    uint8_t (*init)(const void *ctx);
    uint8_t (*get_level)(const void *ctx);
} gas_sensor_driver_t;

extern const gas_sensor_driver_t gas_sensor_driver;

#endif /* _DRV_GAS_SENSOR_H_ */
