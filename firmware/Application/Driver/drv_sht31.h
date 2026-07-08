#ifndef _DRV_SHT31_H_
#define _DRV_SHT31_H_

#include <stdint.h>

typedef struct {
    uint16_t temperature_raw;
    uint16_t humidity_raw;
} sht31_raw_data_t;

typedef struct {
    const void *ctx;
    uint8_t (*init)(const void *ctx);
    uint8_t (*start_measurement)(const void *ctx);
    uint8_t (*read_measurement)(const void *ctx, sht31_raw_data_t *data);
} sht31_driver_t;

extern const sht31_driver_t sht31_driver;

#endif /* _DRV_SHT31_H_ */
