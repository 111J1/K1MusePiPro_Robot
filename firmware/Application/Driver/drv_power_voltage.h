#ifndef _DRV_POWER_VOLTAGE_H_
#define _DRV_POWER_VOLTAGE_H_

#include <stdint.h>

typedef struct {
    const void *ctx;
    uint8_t (*init)(const void *ctx);
    uint8_t (*read_raw)(const void *ctx, uint16_t *raw);
} power_voltage_driver_t;

extern const power_voltage_driver_t power_voltage_driver;

#endif /* _DRV_POWER_VOLTAGE_H_ */
