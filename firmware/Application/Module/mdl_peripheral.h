#ifndef _MDL_PERIPHERAL_H_
#define _MDL_PERIPHERAL_H_

#include <stdint.h>

typedef enum {
    PERIPHERAL_POWER_NORMAL = 0,
    PERIPHERAL_POWER_LOW,
    PERIPHERAL_POWER_CRITICAL,
    PERIPHERAL_POWER_FAULT,
} peripheral_power_state_e;

typedef enum {
    PERIPHERAL_SHT31_STATUS_ERR = 0,
    PERIPHERAL_SHT31_STATUS_WAIT,
    PERIPHERAL_SHT31_STATUS_OK,
} peripheral_sht31_status_e;

typedef struct {
    void *ctx;
    void (*init)(void *ctx);
    void (*update)(void *ctx);
    uint8_t (*get_level)(void *ctx);
    uint8_t (*is_initialized)(void *ctx);
} peripheral_gas_sensor_t;

typedef struct {
    void *ctx;
    void (*init)(void *ctx);
    void (*update)(void *ctx);
    uint8_t (*is_initialized)(void *ctx);
    uint8_t (*is_valid)(void *ctx);
    uint16_t (*get_power_mv)(void *ctx);
} peripheral_power_voltage_t;

typedef struct {
    void *ctx;
    void (*init)(void *ctx);
    void (*update)(void *ctx, uint32_t dt_ms);
    uint8_t (*is_initialized)(void *ctx);
    uint8_t (*is_valid)(void *ctx);
    uint8_t (*get_status)(void *ctx);
    int16_t (*get_temperature_centi_c)(void *ctx);
    uint16_t (*get_humidity_centi_pct)(void *ctx);
} peripheral_sht31_t;

typedef struct {
    uint8_t gas_detected;
    uint8_t gas_level;
    uint8_t gas_initialized;
    uint8_t power_initialized;
    uint8_t power_valid;
    uint8_t sht31_initialized;
    uint8_t sht31_valid;
    peripheral_sht31_status_e sht31_status;
    peripheral_power_state_e power_state;
    peripheral_power_state_e power_pending_state;
    uint32_t power_pending_elapsed_ms;
    uint16_t power_mv;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_pct;
} peripheral_state_t;

typedef struct {
    peripheral_gas_sensor_t gas_sensor;
    peripheral_power_voltage_t power_voltage;
    peripheral_sht31_t sht31;
    peripheral_state_t state;
    peripheral_power_state_e power_pending_state;
    uint32_t power_pending_elapsed_ms;
} peripheral_t;

void peripheral_init(peripheral_t *peripheral);
void peripheral_update(peripheral_t *peripheral, uint32_t dt_ms);
const peripheral_state_t *peripheral_get_state(const peripheral_t *peripheral);

#endif /* _MDL_PERIPHERAL_H_ */
