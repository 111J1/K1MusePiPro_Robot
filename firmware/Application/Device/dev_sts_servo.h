#ifndef _DEV_STS_SERVO_H_
#define _DEV_STS_SERVO_H_

#include <stdint.h>

#include "dev_sts_servo_config.h"
#include "dev_uart.h"

typedef enum {
    STS_SERVO_OK = 0,
    STS_SERVO_BUSY,
    STS_SERVO_PENDING,
    STS_SERVO_ERR_NULL,
    STS_SERVO_ERR_PARAM,
    STS_SERVO_ERR_RANGE,
    STS_SERVO_ERR_TX,
    STS_SERVO_ERR_RX,
    STS_SERVO_ERR_TIMEOUT,
    STS_SERVO_ERR_CHECKSUM,
    STS_SERVO_ERR_PACKET,
    STS_SERVO_ERR_ID,
    STS_SERVO_ERR_STATUS,
    STS_SERVO_ERR_OVERFLOW,
} sts_servo_status_t;

typedef struct {
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[STS_SERVO_MAX_PARAM_LEN];
    uint8_t param_len;
    uint8_t checksum;
} sts_servo_response_t;

typedef struct {
    uint8_t id;
    uint8_t instruction;
    uint8_t params[STS_SERVO_MAX_PARAM_LEN];
    uint8_t param_len;
    uint8_t expect_response;
} sts_servo_command_t;

typedef enum {
    STS_SERVO_PARSER_WAIT_HEADER_1 = 0,
    STS_SERVO_PARSER_WAIT_HEADER_2,
    STS_SERVO_PARSER_ID,
    STS_SERVO_PARSER_LENGTH,
    STS_SERVO_PARSER_ERROR,
    STS_SERVO_PARSER_PARAM,
    STS_SERVO_PARSER_CHECKSUM,
} sts_servo_parser_state_t;

typedef struct sts_servo_bus_class {
    const uart_driver_t *uart;

    uint8_t tx_buf[STS_SERVO_TX_BUF_SIZE];
    uint8_t tx_busy;  /* Bus transaction active, not physical UART TX state. */

    sts_servo_command_t queue[STS_SERVO_CMD_QUEUE_SIZE];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;

    sts_servo_parser_state_t parser_state;
    uint8_t parser_param_index;
    uint8_t parser_sum;

    uint8_t expected_id;
    uint8_t pending_response_count;

    uint32_t timeout_ms;
    uint32_t deadline_ms;

    sts_servo_response_t response;
    sts_servo_response_t response_queue[STS_SERVO_RSP_QUEUE_SIZE];
    uint8_t response_head;
    uint8_t response_tail;
    uint8_t response_count;
    sts_servo_status_t last_status;
} sts_servo_bus_t;

typedef struct sts_servo_class {
    uint8_t id;

    float target_angle_rad;
    float current_angle_rad;

    int16_t target_position;
    int16_t current_position;
    uint16_t current_speed;
    uint16_t current_load;
    uint8_t current_voltage;
    uint8_t current_temperature;
    uint8_t status_flags;
    uint8_t moving;
    uint16_t current_current;

    uint8_t target_dirty;
    uint8_t feedback_dirty;
    uint8_t is_initialized;
    uint8_t is_online;
    uint8_t torque_enabled;
    uint8_t protection_enabled;
    uint8_t protection_active;

    float angle_min_rad;
    float angle_max_rad;
    int16_t position_min;
    int16_t position_max;
    int16_t position_zero;
    int8_t direction;
    float position_per_rad;

    uint16_t default_speed;
    uint16_t command_time;
    uint16_t command_speed;
    uint8_t default_acc;
    uint32_t timeout_ms;

    uint32_t fault_flags;
    uint8_t max_temperature;
    uint8_t min_voltage;
    uint8_t max_voltage;
    uint16_t max_load;
    uint16_t max_current;
    uint16_t max_position_error;

    sts_servo_bus_t *bus;
    sts_servo_status_t last_status;
} sts_servo_t;

void sts_servo_bus_init(sts_servo_bus_t *bus, const uart_driver_t *uart);

sts_servo_status_t sts_servo_write_command(sts_servo_bus_t *bus,
                                           uint8_t id,
                                           uint8_t instruction,
                                           const uint8_t *params,
                                           uint8_t param_len,
                                           uint8_t expect_response);
sts_servo_status_t sts_servo_parse_response(sts_servo_bus_t *bus);
sts_servo_status_t sts_servo_bus_update(sts_servo_bus_t *bus);

sts_servo_status_t sts_servo_ping(sts_servo_bus_t *bus, uint8_t id);
sts_servo_status_t sts_servo_read_mem(sts_servo_bus_t *bus,
                                      uint8_t id,
                                      uint8_t addr,
                                      uint8_t len);
sts_servo_status_t sts_servo_write_mem(sts_servo_bus_t *bus,
                                       uint8_t id,
                                       uint8_t addr,
                                       const uint8_t *data,
                                       uint8_t len);
sts_servo_status_t sts_servo_reg_write_mem(sts_servo_bus_t *bus,
                                           uint8_t id,
                                           uint8_t addr,
                                           const uint8_t *data,
                                           uint8_t len);
sts_servo_status_t sts_servo_action(sts_servo_bus_t *bus);
sts_servo_status_t sts_servo_sync_write_mem(sts_servo_bus_t *bus,
                                            uint8_t addr,
                                            uint8_t data_len,
                                            const uint8_t *items,
                                            uint8_t item_len);
sts_servo_status_t sts_servo_sync_read_mem(sts_servo_bus_t *bus,
                                           uint8_t addr,
                                           uint8_t data_len,
                                           const uint8_t *ids,
                                           uint8_t id_count);
sts_servo_status_t sts_servo_pop_response(sts_servo_bus_t *bus,
                                          sts_servo_response_t *response);

// interface functions
void sts_servo_init(void *ctx);
void sts_servo_set_angle_rad(void *ctx, float angle_rad);
float sts_servo_get_angle_rad(void *ctx);
uint32_t sts_servo_get_faults(void *ctx);
void sts_servo_enable_torque(void *ctx, uint8_t enable);
void sts_servo_update(void *ctx);
sts_servo_status_t sts_servo_update_sync(sts_servo_bus_t *bus,
                                         sts_servo_t *servos,
                                         uint8_t servo_count);

int16_t sts_servo_angle_to_position(const sts_servo_t *servo, float angle_rad);
float sts_servo_position_to_angle(const sts_servo_t *servo, int16_t position);

void sts_servo_put_u16_le(uint8_t *buf, uint16_t value);
uint16_t sts_servo_get_u16_le(const uint8_t *buf);

#endif /* _DEV_STS_SERVO_H_ */
