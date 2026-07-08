#include "dev_sts_servo.h"

#define STS_SERVO_MIN_PACKET_LEN (6U)
#define STS_SERVO_STATUS_READ_ADDR STS_SERVO_ADDR_CURRENT_POSITION
#define STS_SERVO_STATUS_READ_LEN ((uint8_t)(STS_SERVO_ADDR_CURRENT_CURRENT - STS_SERVO_ADDR_CURRENT_POSITION + 2U))
#define STS_SERVO_LOAD_VALUE_MASK (0x03FFU)

__attribute__((weak)) uint32_t sts_servo_platform_get_tick_ms(void)
{
    return 0U;
}

static int16_t sts_servo_constrain_i16(int16_t value, int16_t min, int16_t max)
{
    return (value > max) ? max : ((value < min) ? min : value);
}

static float sts_servo_constrain_float(float value, float min, float max)
{
    return (value > max) ? max : ((value < min) ? min : value);
}

static sts_servo_t *sts_servo_find_by_id(sts_servo_t *servos, uint8_t servo_count, uint8_t id)
{
    if (servos == 0) {
        return 0;
    }

    for (uint8_t i = 0U; i < servo_count; i++) {
        if (servos[i].id == id) {
            return &servos[i];
        }
    }

    return 0;
}

static uint16_t sts_servo_abs_position_error(const sts_servo_t *servo)
{
    int16_t error;

    if (servo == 0) {
        return 0U;
    }

    error = (int16_t)(servo->target_position - servo->current_position);
    return (error < 0) ? (uint16_t)(-error) : (uint16_t)error;
}

static void sts_servo_update_fault_flags(sts_servo_t *servo)
{
    uint32_t faults = STS_SERVO_FAULT_NONE;
    uint16_t load_value;

    if (servo == 0) {
        return;
    }

    if ((servo->current_temperature != 0U) &&
        (servo->current_temperature > servo->max_temperature)) {
        faults |= STS_SERVO_FAULT_OVER_TEMP;
    }
    if ((servo->current_voltage != 0U) &&
        (servo->current_voltage < servo->min_voltage)) {
        faults |= STS_SERVO_FAULT_UNDER_VOLTAGE;
    }
    if ((servo->current_voltage != 0U) &&
        (servo->current_voltage > servo->max_voltage)) {
        faults |= STS_SERVO_FAULT_OVER_VOLTAGE;
    }

    load_value = (uint16_t)(servo->current_load & STS_SERVO_LOAD_VALUE_MASK);
    if ((servo->max_load != 0U) && (load_value > servo->max_load)) {
        faults |= STS_SERVO_FAULT_OVER_LOAD;
    }
    if ((servo->max_current != 0U) &&
        (servo->current_current != 0U) &&
        (servo->current_current > servo->max_current)) {
        faults |= STS_SERVO_FAULT_OVER_CURRENT;
    }
    if ((servo->max_position_error != 0U) &&
        (sts_servo_abs_position_error(servo) > servo->max_position_error)) {
        faults |= STS_SERVO_FAULT_POSITION_ERROR;
    }
    if (servo->last_status == STS_SERVO_ERR_TIMEOUT) {
        faults |= STS_SERVO_FAULT_BUS_TIMEOUT;
    }
    if ((servo->status_flags != 0U) || (servo->last_status == STS_SERVO_ERR_STATUS)) {
        faults |= STS_SERVO_FAULT_STATUS_ERROR;
    }

    servo->fault_flags = faults;
}

static uint8_t sts_servo_should_disable_torque(const sts_servo_t *servo)
{
    if ((servo == 0) || (servo->protection_enabled == 0U)) {
        return 0U;
    }

    return ((servo->fault_flags & STS_SERVO_PROTECT_DISABLE_TORQUE_FAULTS) != 0UL) ? 1U : 0U;
}

static uint8_t sts_servo_packet_checksum(uint8_t id,
                                         uint8_t length,
                                         uint8_t instruction_or_error,
                                         const uint8_t *params,
                                         uint8_t param_len)
{
    uint16_t sum = (uint16_t)id + length + instruction_or_error;

    for (uint8_t i = 0U; i < param_len; i++) {
        sum += params[i];
    }

    return (uint8_t)(~((uint8_t)sum));
}

static void sts_servo_parser_reset(sts_servo_bus_t *bus)
{
    bus->parser_state = STS_SERVO_PARSER_WAIT_HEADER_1;
    bus->parser_param_index = 0U;
    bus->parser_sum = 0U;
    bus->response.id = 0U;
    bus->response.length = 0U;
    bus->response.error = 0U;
    bus->response.param_len = 0U;
    bus->response.checksum = 0U;
}

static uint8_t sts_servo_queue_push(sts_servo_bus_t *bus, const sts_servo_command_t *cmd)
{
    if ((bus == 0) || (cmd == 0) || (bus->queue_count >= STS_SERVO_CMD_QUEUE_SIZE)) {
        return 0U;
    }

    bus->queue[bus->queue_head] = *cmd;
    bus->queue_head = (uint8_t)((bus->queue_head + 1U) % STS_SERVO_CMD_QUEUE_SIZE);
    bus->queue_count++;
    return 1U;
}

static uint8_t sts_servo_queue_pop(sts_servo_bus_t *bus, sts_servo_command_t *cmd)
{
    if ((bus == 0) || (cmd == 0) || (bus->queue_count == 0U)) {
        return 0U;
    }

    *cmd = bus->queue[bus->queue_tail];
    bus->queue_tail = (uint8_t)((bus->queue_tail + 1U) % STS_SERVO_CMD_QUEUE_SIZE);
    bus->queue_count--;
    return 1U;
}

static uint8_t sts_servo_response_push(sts_servo_bus_t *bus, const sts_servo_response_t *response)
{
    if ((bus == 0) || (response == 0) || (bus->response_count >= STS_SERVO_RSP_QUEUE_SIZE)) {
        return 0U;
    }

    bus->response_queue[bus->response_head] = *response;
    bus->response_head = (uint8_t)((bus->response_head + 1U) % STS_SERVO_RSP_QUEUE_SIZE);
    bus->response_count++;
    return 1U;
}

sts_servo_status_t sts_servo_pop_response(sts_servo_bus_t *bus, sts_servo_response_t *response)
{
    if ((bus == 0) || (response == 0)) {
        return STS_SERVO_ERR_NULL;
    }
    if (bus->response_count == 0U) {
        return STS_SERVO_PENDING;
    }

    *response = bus->response_queue[bus->response_tail];
    bus->response_tail = (uint8_t)((bus->response_tail + 1U) % STS_SERVO_RSP_QUEUE_SIZE);
    bus->response_count--;
    return STS_SERVO_OK;
}

static sts_servo_status_t sts_servo_encode_command(sts_servo_bus_t *bus,
                                                   const sts_servo_command_t *cmd,
                                                   uint16_t *packet_len)
{
    uint8_t length;

    if ((bus == 0) || (cmd == 0) || (packet_len == 0)) {
        return STS_SERVO_ERR_NULL;
    }

    length = (uint8_t)(cmd->param_len + 2U);
    *packet_len = (uint16_t)cmd->param_len + STS_SERVO_MIN_PACKET_LEN;
    if (*packet_len > STS_SERVO_TX_BUF_SIZE) {
        return STS_SERVO_ERR_OVERFLOW;
    }

    bus->tx_buf[0] = STS_SERVO_HEADER_1;
    bus->tx_buf[1] = STS_SERVO_HEADER_2;
    bus->tx_buf[2] = cmd->id;
    bus->tx_buf[3] = length;
    bus->tx_buf[4] = cmd->instruction;

    for (uint8_t i = 0U; i < cmd->param_len; i++) {
        bus->tx_buf[5U + i] = cmd->params[i];
    }

    bus->tx_buf[5U + cmd->param_len] =
        sts_servo_packet_checksum(cmd->id, length, cmd->instruction, cmd->params, cmd->param_len);

    return STS_SERVO_OK;
}

static sts_servo_status_t sts_servo_send_next_command(sts_servo_bus_t *bus)
{
    sts_servo_command_t cmd;
    sts_servo_status_t status;
    uint16_t packet_len;

    if ((bus == 0) || (bus->uart == 0) || (bus->uart->send == 0)) {
        return STS_SERVO_ERR_NULL;
    }
    if ((bus->pending_response_count != 0U) || (bus->tx_busy != 0U) || (bus->queue_count == 0U)) {
        return STS_SERVO_PENDING;
    }
    if (sts_servo_queue_pop(bus, &cmd) == 0U) {
        return STS_SERVO_PENDING;
    }

    status = sts_servo_encode_command(bus, &cmd, &packet_len);
    if (status != STS_SERVO_OK) {
        bus->last_status = status;
        return status;
    }

    if (bus->uart->send(bus->uart->ctx, bus->tx_buf, packet_len) == 0U) {
        bus->last_status = STS_SERVO_ERR_TX;
        return STS_SERVO_ERR_TX;
    }

    bus->tx_busy = 1U;
    bus->expected_id = cmd.id;
    if ((cmd.instruction == STS_SERVO_INST_SYNC_READ) && (cmd.param_len >= 3U)) {
        bus->pending_response_count = (uint8_t)(cmd.param_len - 2U);
    } else {
        bus->pending_response_count = (cmd.expect_response != 0U) ? 1U : 0U;
    }

    if (bus->pending_response_count != 0U) {
        bus->deadline_ms = sts_servo_platform_get_tick_ms() + bus->timeout_ms;
    } else {
        bus->last_status = STS_SERVO_OK;
    }

    return STS_SERVO_OK;
}

void sts_servo_bus_init(sts_servo_bus_t *bus, const uart_driver_t *uart)
{
    if (bus == 0) {
        return;
    }

    bus->uart = uart;
    bus->tx_busy = 0U;
    bus->queue_head = 0U;
    bus->queue_tail = 0U;
    bus->queue_count = 0U;
    bus->expected_id = 0U;
    bus->pending_response_count = 0U;
    bus->timeout_ms = STS_SERVO_DEFAULT_TIMEOUT_MS;
    bus->deadline_ms = 0U;
    bus->response_head = 0U;
    bus->response_tail = 0U;
    bus->response_count = 0U;
    bus->last_status = STS_SERVO_OK;
    sts_servo_parser_reset(bus);

    if ((uart != 0) && (uart->init != 0)) {
        if (uart->init(uart->ctx) == 0U) {
            bus->last_status = STS_SERVO_ERR_TX;
        }
    }
}

sts_servo_status_t sts_servo_write_command(sts_servo_bus_t *bus,
                                           uint8_t id,
                                           uint8_t instruction,
                                           const uint8_t *params,
                                           uint8_t param_len,
                                           uint8_t expect_response)
{
    sts_servo_command_t cmd;

    if (bus == 0) {
        return STS_SERVO_ERR_NULL;
    }
    if ((id > STS_SERVO_BROADCAST_ID) ||
        ((param_len > 0U) && (params == 0)) ||
        (param_len > STS_SERVO_MAX_PARAM_LEN)) {
        return STS_SERVO_ERR_PARAM;
    }
    if (bus->queue_count >= STS_SERVO_CMD_QUEUE_SIZE) {
        return STS_SERVO_BUSY;
    }

    cmd.id = id;
    cmd.instruction = instruction;
    cmd.param_len = param_len;
    cmd.expect_response = expect_response;
    for (uint8_t i = 0U; i < param_len; i++) {
        cmd.params[i] = params[i];
    }

    if (sts_servo_queue_push(bus, &cmd) == 0U) {
        return STS_SERVO_BUSY;
    }

    return STS_SERVO_OK;
}

static sts_servo_status_t sts_servo_parse_byte(sts_servo_bus_t *bus, uint8_t byte)
{
    sts_servo_response_t *rsp = &bus->response;

    switch (bus->parser_state) {
    case STS_SERVO_PARSER_WAIT_HEADER_1:
        if (byte == STS_SERVO_HEADER_1) {
            bus->parser_state = STS_SERVO_PARSER_WAIT_HEADER_2;
        }
        break;

    case STS_SERVO_PARSER_WAIT_HEADER_2:
        if (byte == STS_SERVO_HEADER_2) {
            bus->parser_state = STS_SERVO_PARSER_ID;
        } else if (byte != STS_SERVO_HEADER_1) {
            bus->parser_state = STS_SERVO_PARSER_WAIT_HEADER_1;
        }
        break;

    case STS_SERVO_PARSER_ID:
        rsp->id = byte;
        bus->parser_sum = byte;
        bus->parser_state = STS_SERVO_PARSER_LENGTH;
        break;

    case STS_SERVO_PARSER_LENGTH:
        rsp->length = byte;
        bus->parser_sum = (uint8_t)(bus->parser_sum + byte);
        if ((byte < 2U) || ((uint16_t)(byte - 2U) > STS_SERVO_MAX_PARAM_LEN)) {
            sts_servo_parser_reset(bus);
            bus->last_status = STS_SERVO_ERR_PACKET;
            return STS_SERVO_ERR_PACKET;
        }
        bus->parser_state = STS_SERVO_PARSER_ERROR;
        break;

    case STS_SERVO_PARSER_ERROR:
        rsp->error = byte;
        bus->parser_sum = (uint8_t)(bus->parser_sum + byte);
        bus->parser_param_index = 0U;
        if (rsp->length > 2U) {
            bus->parser_state = STS_SERVO_PARSER_PARAM;
        } else {
            bus->parser_state = STS_SERVO_PARSER_CHECKSUM;
        }
        break;

    case STS_SERVO_PARSER_PARAM:
        rsp->params[bus->parser_param_index++] = byte;
        bus->parser_sum = (uint8_t)(bus->parser_sum + byte);
        if (bus->parser_param_index >= (uint8_t)(rsp->length - 2U)) {
            bus->parser_state = STS_SERVO_PARSER_CHECKSUM;
        }
        break;

    case STS_SERVO_PARSER_CHECKSUM:
    {
        uint8_t expected_checksum = (uint8_t)(~bus->parser_sum);
        rsp->checksum = byte;
        if (expected_checksum != byte) {
            sts_servo_parser_reset(bus);
            bus->last_status = STS_SERVO_ERR_CHECKSUM;
            return STS_SERVO_ERR_CHECKSUM;
        }

        rsp->param_len = (uint8_t)(rsp->length - 2U);

        if ((bus->pending_response_count != 0U) &&
            (bus->expected_id != STS_SERVO_BROADCAST_ID) &&
            (rsp->id != bus->expected_id)) {
            sts_servo_parser_reset(bus);
            return STS_SERVO_PENDING;
        }

        if (sts_servo_response_push(bus, rsp) == 0U) {
            sts_servo_parser_reset(bus);
            bus->last_status = STS_SERVO_ERR_OVERFLOW;
            return STS_SERVO_ERR_OVERFLOW;
        }

        if (bus->pending_response_count != 0U) {
            bus->pending_response_count--;
            if (bus->pending_response_count == 0U) {
                bus->tx_busy = 0U;
            }
        } else {
            bus->tx_busy = 0U;
        }
        bus->last_status = (rsp->error == 0U) ? STS_SERVO_OK : STS_SERVO_ERR_STATUS;
        sts_servo_parser_reset(bus);
        return bus->last_status;
    }

    default:
        sts_servo_parser_reset(bus);
        break;
    }

    return STS_SERVO_PENDING;
}

sts_servo_status_t sts_servo_parse_response(sts_servo_bus_t *bus)
{
    uint8_t rx_tmp[STS_SERVO_RX_TMP_BUF_SIZE];
    uint16_t available;
    sts_servo_status_t status = STS_SERVO_PENDING;

    if ((bus == 0) || (bus->uart == 0) || (bus->uart->available == 0) || (bus->uart->read == 0)) {
        return STS_SERVO_ERR_NULL;
    }

    available = bus->uart->available(bus->uart->ctx);
    while (available > 0U) {
        uint16_t chunk = (available > STS_SERVO_RX_TMP_BUF_SIZE) ? STS_SERVO_RX_TMP_BUF_SIZE : available;
        uint16_t read_len = bus->uart->read(bus->uart->ctx, rx_tmp, chunk);

        if (read_len == 0U) {
            break;
        }

        for (uint16_t i = 0U; i < read_len; i++) {
            status = sts_servo_parse_byte(bus, rx_tmp[i]);
        }

        available = bus->uart->available(bus->uart->ctx);
    }

    return status;
}

sts_servo_status_t sts_servo_bus_update(sts_servo_bus_t *bus)
{
    sts_servo_status_t status;
    uint32_t now;

    if (bus == 0) {
        return STS_SERVO_ERR_NULL;
    }

    status = sts_servo_parse_response(bus);
    if ((status == STS_SERVO_OK) || (status == STS_SERVO_ERR_STATUS)) {
        if (bus->pending_response_count == 0U) {
            return status;
        }
        return STS_SERVO_PENDING;
    }

    if (bus->pending_response_count != 0U) {
        now = sts_servo_platform_get_tick_ms();
        if ((int32_t)(now - bus->deadline_ms) >= 0) {
            bus->pending_response_count = 0U;
            bus->tx_busy = 0U;
            sts_servo_parser_reset(bus);
            bus->last_status = STS_SERVO_ERR_TIMEOUT;
            return STS_SERVO_ERR_TIMEOUT;
        }
        return STS_SERVO_PENDING;
    }

    if (bus->tx_busy != 0U) {
        bus->tx_busy = 0U;
    }

    return sts_servo_send_next_command(bus);
}

sts_servo_status_t sts_servo_ping(sts_servo_bus_t *bus, uint8_t id)
{
    return sts_servo_write_command(bus, id, STS_SERVO_INST_PING, 0, 0U, 1U);
}

sts_servo_status_t sts_servo_read_mem(sts_servo_bus_t *bus, uint8_t id, uint8_t addr, uint8_t len)
{
    uint8_t params[2];

    if (len == 0U) {
        return STS_SERVO_ERR_PARAM;
    }

    params[0] = addr;
    params[1] = len;
    return sts_servo_write_command(bus, id, STS_SERVO_INST_READ, params, 2U, 1U);
}

static sts_servo_status_t sts_servo_write_mem_inst(sts_servo_bus_t *bus,
                                                   uint8_t id,
                                                   uint8_t instruction,
                                                   uint8_t addr,
                                                   const uint8_t *data,
                                                   uint8_t len)
{
    uint8_t params[STS_SERVO_MAX_PARAM_LEN];
    uint8_t expect_response;

    if ((len == 0U) || (data == 0) || ((uint16_t)len + 1U > STS_SERVO_MAX_PARAM_LEN)) {
        return STS_SERVO_ERR_PARAM;
    }

    params[0] = addr;
    for (uint8_t i = 0U; i < len; i++) {
        params[1U + i] = data[i];
    }

    expect_response = (id == STS_SERVO_BROADCAST_ID) ? 0U : 1U;
    return sts_servo_write_command(bus, id, instruction, params, (uint8_t)(len + 1U), expect_response);
}

sts_servo_status_t sts_servo_write_mem(sts_servo_bus_t *bus,
                                       uint8_t id,
                                       uint8_t addr,
                                       const uint8_t *data,
                                       uint8_t len)
{
    return sts_servo_write_mem_inst(bus, id, STS_SERVO_INST_WRITE, addr, data, len);
}

sts_servo_status_t sts_servo_reg_write_mem(sts_servo_bus_t *bus,
                                           uint8_t id,
                                           uint8_t addr,
                                           const uint8_t *data,
                                           uint8_t len)
{
    return sts_servo_write_mem_inst(bus, id, STS_SERVO_INST_REG_WRITE, addr, data, len);
}

sts_servo_status_t sts_servo_action(sts_servo_bus_t *bus)
{
    return sts_servo_write_command(bus, STS_SERVO_BROADCAST_ID, STS_SERVO_INST_ACTION, 0, 0U, 0U);
}

sts_servo_status_t sts_servo_sync_write_mem(sts_servo_bus_t *bus,
                                            uint8_t addr,
                                            uint8_t data_len,
                                            const uint8_t *items,
                                            uint8_t item_len)
{
    uint8_t params[STS_SERVO_MAX_PARAM_LEN];

    if ((data_len == 0U) || (item_len == 0U) || (items == 0) ||
        ((uint16_t)item_len + 2U > STS_SERVO_MAX_PARAM_LEN)) {
        return STS_SERVO_ERR_PARAM;
    }

    params[0] = addr;
    params[1] = data_len;
    for (uint8_t i = 0U; i < item_len; i++) {
        params[2U + i] = items[i];
    }

    return sts_servo_write_command(bus,
                                   STS_SERVO_BROADCAST_ID,
                                   STS_SERVO_INST_SYNC_WRITE,
                                   params,
                                   (uint8_t)(item_len + 2U),
                                   0U);
}

sts_servo_status_t sts_servo_sync_read_mem(sts_servo_bus_t *bus,
                                           uint8_t addr,
                                           uint8_t data_len,
                                           const uint8_t *ids,
                                           uint8_t id_count)
{
    uint8_t params[STS_SERVO_MAX_PARAM_LEN];

    if ((data_len == 0U) || (id_count == 0U) || (ids == 0) ||
        ((uint16_t)id_count + 2U > STS_SERVO_MAX_PARAM_LEN)) {
        return STS_SERVO_ERR_PARAM;
    }

    params[0] = addr;
    params[1] = data_len;
    for (uint8_t i = 0U; i < id_count; i++) {
        params[2U + i] = ids[i];
    }

    return sts_servo_write_command(bus,
                                   STS_SERVO_BROADCAST_ID,
                                   STS_SERVO_INST_SYNC_READ,
                                   params,
                                   (uint8_t)(id_count + 2U),
                                   1U);
}

void sts_servo_init(void *ctx)
{
    sts_servo_t *servo = (sts_servo_t *)ctx;

    if (servo == 0) {
        return;
    }

    if (servo->position_per_rad == 0.0f) {
        servo->position_per_rad = STS_SERVO_POSITION_PER_RAD_DEFAULT;
    }
    if ((servo->position_min == 0) && (servo->position_max == 0)) {
        servo->position_min = STS_SERVO_POSITION_MIN_DEFAULT;
        servo->position_max = STS_SERVO_POSITION_MAX_DEFAULT;
    }
    if (servo->position_zero == 0) {
        servo->position_zero = STS_SERVO_POSITION_ZERO_DEFAULT;
    }
    if (servo->direction == 0) {
        servo->direction = 1;
    }
    if (servo->default_speed == 0U) {
        servo->default_speed = STS_SERVO_SPEED_DEFAULT;
    }
    if (servo->command_speed == 0U) {
        servo->command_speed = servo->default_speed;
    }
    if (servo->default_acc == 0U) {
        servo->default_acc = STS_SERVO_ACC_DEFAULT;
    }
    if (servo->timeout_ms == 0U) {
        servo->timeout_ms = STS_SERVO_DEFAULT_TIMEOUT_MS;
    }
    if (servo->max_temperature == 0U) {
        servo->max_temperature = STS_SERVO_MAX_TEMPERATURE_DEFAULT;
    }
    if (servo->min_voltage == 0U) {
        servo->min_voltage = STS_SERVO_MIN_VOLTAGE_DEFAULT;
    }
    if (servo->max_voltage == 0U) {
        servo->max_voltage = STS_SERVO_MAX_VOLTAGE_DEFAULT;
    }
    if (servo->max_load == 0U) {
        servo->max_load = STS_SERVO_MAX_LOAD_DEFAULT;
    }
    if (servo->max_current == 0U) {
        servo->max_current = STS_SERVO_MAX_CURRENT_DEFAULT;
    }
    if (servo->max_position_error == 0U) {
        servo->max_position_error = STS_SERVO_MAX_POSITION_ERROR_DEFAULT;
    }
    if (servo->angle_min_rad == servo->angle_max_rad) {
        servo->angle_min_rad = -3.1415926f;
        servo->angle_max_rad = 3.1415926f;
    }

    servo->target_position = sts_servo_angle_to_position(servo, servo->target_angle_rad);
    servo->current_position = servo->target_position;
    servo->current_angle_rad = sts_servo_position_to_angle(servo, servo->current_position);
    servo->current_speed = 0U;
    servo->current_load = 0U;
    servo->current_voltage = 0U;
    servo->current_temperature = 0U;
    servo->status_flags = 0U;
    servo->moving = 0U;
    servo->current_current = 0U;
    servo->fault_flags = STS_SERVO_FAULT_NONE;
    servo->target_dirty = 1U;
    servo->feedback_dirty = 0U;
    servo->is_initialized = (servo->bus != 0) ? 1U : 0U;
    servo->protection_enabled = 1U;
    servo->protection_active = 0U;
    servo->last_status = servo->is_initialized ? STS_SERVO_OK : STS_SERVO_ERR_NULL;

    if (servo->is_initialized != 0U) {
        servo->last_status = sts_servo_ping(servo->bus, servo->id);
    }
}

void sts_servo_set_angle_rad(void *ctx, float angle_rad)
{
    sts_servo_t *servo = (sts_servo_t *)ctx;

    if (servo == 0) {
        return;
    }

    angle_rad = sts_servo_constrain_float(angle_rad, servo->angle_min_rad, servo->angle_max_rad);
    servo->target_angle_rad = angle_rad;
    servo->target_position = sts_servo_angle_to_position(servo, angle_rad);
    servo->target_dirty = 1U;
}

float sts_servo_get_angle_rad(void *ctx)
{
    sts_servo_t *servo = (sts_servo_t *)ctx;

    return (servo != 0) ? servo->current_angle_rad : 0.0f;
}

uint32_t sts_servo_get_faults(void *ctx)
{
    sts_servo_t *servo = (sts_servo_t *)ctx;

    return (servo != 0) ? servo->fault_flags : 0UL;
}

void sts_servo_enable_torque(void *ctx, uint8_t enable)
{
    sts_servo_t *servo = (sts_servo_t *)ctx;
    uint8_t value;

    if ((servo == 0) || (servo->bus == 0)) {
        return;
    }

    value = (enable == 0U) ? STS_SERVO_TORQUE_DISABLE : STS_SERVO_TORQUE_ENABLE;
    servo->torque_enabled = (enable == 0U) ? 0U : 1U;
    if (enable != 0U) {
        servo->protection_active = 0U;
    }
    servo->last_status = sts_servo_write_mem(servo->bus,
                                             servo->id,
                                             STS_SERVO_ADDR_TORQUE_ENABLE,
                                             &value,
                                             1U);
}

void sts_servo_update(void *ctx)
{
    sts_servo_t *servo = (sts_servo_t *)ctx;

    if ((servo == 0) || (servo->bus == 0) || (servo->is_initialized == 0U)) {
        return;
    }

    (void)sts_servo_update_sync(servo->bus, servo, 1U);
}

sts_servo_status_t sts_servo_update_sync(sts_servo_bus_t *bus,
                                         sts_servo_t *servos,
                                         uint8_t servo_count)
{
    sts_servo_response_t response;
    uint8_t sync_write_items[STS_SERVO_MAX_PARAM_LEN];
    uint8_t protect_items[STS_SERVO_MAX_PARAM_LEN];
    uint8_t sync_read_ids[STS_SERVO_MAX_PARAM_LEN];
    uint8_t write_item_len = 0U;
    uint8_t protect_item_len = 0U;
    uint8_t read_id_count = 0U;
    uint8_t dirty_count = 0U;
    uint8_t protect_count = 0U;
    uint8_t queue_needed;
    sts_servo_status_t status;

    if ((bus == 0) || (servos == 0) || (servo_count == 0U)) {
        return STS_SERVO_ERR_NULL;
    }

    status = sts_servo_bus_update(bus);
    if (status == STS_SERVO_ERR_TIMEOUT) {
        for (uint8_t i = 0U; i < servo_count; i++) {
            if (servos[i].is_initialized != 0U) {
                servos[i].last_status = STS_SERVO_ERR_TIMEOUT;
                sts_servo_update_fault_flags(&servos[i]);
            }
        }
    }

    while (sts_servo_pop_response(bus, &response) == STS_SERVO_OK) {
        sts_servo_t *servo = sts_servo_find_by_id(servos, servo_count, response.id);
        if (servo != 0) {
            servo->last_status = (response.error == 0U) ? STS_SERVO_OK : STS_SERVO_ERR_STATUS;
            if ((response.error == 0U) && (response.param_len >= 2U)) {
                servo->current_position = (int16_t)sts_servo_get_u16_le(response.params);
                servo->current_angle_rad = sts_servo_position_to_angle(servo, servo->current_position);
                if (response.param_len >= STS_SERVO_STATUS_READ_LEN) {
                    servo->current_speed = sts_servo_get_u16_le(&response.params[2]);
                    servo->current_load = sts_servo_get_u16_le(&response.params[4]);
                    servo->current_voltage = response.params[6];
                    servo->current_temperature = response.params[7];
                    servo->status_flags = response.params[9];
                    servo->moving = response.params[10];
                    servo->current_current = sts_servo_get_u16_le(&response.params[13]);
                }
                servo->is_online = 1U;
                servo->feedback_dirty = 1U;
            }
            sts_servo_update_fault_flags(servo);
        }
    }

    if ((bus->pending_response_count != 0U) || (bus->tx_busy != 0U)) {
        return status;
    }

    for (uint8_t i = 0U; i < servo_count; i++) {
        if (servos[i].is_initialized != 0U) {
            if (read_id_count >= STS_SERVO_MAX_PARAM_LEN) {
                return STS_SERVO_ERR_OVERFLOW;
            }
            sync_read_ids[read_id_count++] = servos[i].id;

            if (sts_servo_should_disable_torque(&servos[i]) != 0U) {
                servos[i].target_dirty = 0U;
                if (servos[i].protection_active == 0U) {
                    if ((uint16_t)protect_item_len + 2U > STS_SERVO_MAX_PARAM_LEN) {
                        return STS_SERVO_ERR_OVERFLOW;
                    }
                    protect_items[protect_item_len++] = servos[i].id;
                    protect_items[protect_item_len++] = STS_SERVO_TORQUE_DISABLE;
                    protect_count++;
                }
            } else if ((servos[i].target_dirty != 0U) && (servos[i].protection_active == 0U)) {
                if ((uint16_t)write_item_len + 7U > STS_SERVO_MAX_PARAM_LEN) {
                    return STS_SERVO_ERR_OVERFLOW;
                }
                sync_write_items[write_item_len++] = servos[i].id;
                sts_servo_put_u16_le(&sync_write_items[write_item_len], (uint16_t)servos[i].target_position);
                write_item_len += 2U;
                sts_servo_put_u16_le(&sync_write_items[write_item_len], servos[i].command_time);
                write_item_len += 2U;
                sts_servo_put_u16_le(&sync_write_items[write_item_len], servos[i].command_speed);
                write_item_len += 2U;
                dirty_count++;
            }
        }
    }

    if (read_id_count == 0U) {
        return STS_SERVO_ERR_PARAM;
    }

    queue_needed = 1U;
    if (protect_count > 0U) {
        queue_needed++;
    }
    if (dirty_count > 0U) {
        queue_needed++;
    }
    if ((queue_needed > STS_SERVO_CMD_QUEUE_SIZE) ||
        (bus->queue_count > (uint8_t)(STS_SERVO_CMD_QUEUE_SIZE - queue_needed))) {
        return STS_SERVO_BUSY;
    }

    if (protect_count > 0U) {
        status = sts_servo_sync_write_mem(bus,
                                          STS_SERVO_ADDR_TORQUE_ENABLE,
                                          1U,
                                          protect_items,
                                          protect_item_len);
        if (status != STS_SERVO_OK) {
            return status;
        }
        for (uint8_t i = 0U; i < servo_count; i++) {
            if (sts_servo_should_disable_torque(&servos[i]) != 0U) {
                servos[i].torque_enabled = 0U;
                servos[i].protection_active = 1U;
                servos[i].target_dirty = 0U;
            }
        }
    }

    if (dirty_count > 0U) {
        status = sts_servo_sync_write_mem(bus,
                                          STS_SERVO_ADDR_TARGET_POSITION,
                                          6U,
                                          sync_write_items,
                                          write_item_len);
        if (status != STS_SERVO_OK) {
            return status;
        }
        for (uint8_t i = 0U; i < servo_count; i++) {
            servos[i].target_dirty = 0U;
        }
    }

    status = sts_servo_sync_read_mem(bus,
                                     STS_SERVO_STATUS_READ_ADDR,
                                     STS_SERVO_STATUS_READ_LEN,
                                     sync_read_ids,
                                     read_id_count);
    return status;
}

int16_t sts_servo_angle_to_position(const sts_servo_t *servo, float angle_rad)
{
    float delta;
    int16_t position;

    if (servo == 0) {
        return 0;
    }

    delta = angle_rad * servo->position_per_rad * (float)servo->direction;
    position = (int16_t)((float)servo->position_zero + delta);
    return sts_servo_constrain_i16(position, servo->position_min, servo->position_max);
}

float sts_servo_position_to_angle(const sts_servo_t *servo, int16_t position)
{
    if ((servo == 0) || (servo->position_per_rad == 0.0f) || (servo->direction == 0)) {
        return 0.0f;
    }

    return ((float)(position - servo->position_zero) / servo->position_per_rad) /
           (float)servo->direction;
}

void sts_servo_put_u16_le(uint8_t *buf, uint16_t value)
{
    if (buf == 0) {
        return;
    }

    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

uint16_t sts_servo_get_u16_le(const uint8_t *buf)
{
    if (buf == 0) {
        return 0U;
    }

    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8U);
}
