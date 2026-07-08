#ifndef _DEV_SHT31_H_
#define _DEV_SHT31_H_

#include "drv_sht31.h"
#include <stdint.h>

typedef enum {
    SHT31_STATUS_ERR = 0,
    SHT31_STATUS_WAIT,
    SHT31_STATUS_OK,
} sht31_status_e;

typedef enum {
    SHT31_MEASURE_IDLE = 0,
    SHT31_MEASURE_WAIT,
} sht31_measure_state_e;

typedef struct {
    uint8_t is_initialized;
    uint8_t is_valid;
    uint8_t error_count;
    sht31_status_e status;
    sht31_measure_state_e measure_state;

    uint16_t temperature_raw;
    uint16_t humidity_raw;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_pct;

    uint32_t measure_period_ms;
    uint32_t measure_wait_ms;
    uint8_t error_limit;
    uint32_t period_elapsed_ms;
    uint32_t wait_elapsed_ms;

    const sht31_driver_t *driver;
} sht31_device_t;

void sht31_init(void *ctx);
void sht31_update(void *ctx, uint32_t dt_ms);
uint8_t sht31_is_initialized(void *ctx);
uint8_t sht31_is_valid(void *ctx);
uint8_t sht31_get_status(void *ctx);
int16_t sht31_get_temperature_centi_c(void *ctx);
uint16_t sht31_get_humidity_centi_pct(void *ctx);

#endif /* _DEV_SHT31_H_ */
