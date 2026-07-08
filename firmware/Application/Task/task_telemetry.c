#include "task_telemetry.h"

#include "drv_uart.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#define TELEMETRY_FRAME_BUF_SIZE (2U + 5U + CTRL_PROTOCOL_MAX_PAYLOAD + 1U)
#define TELEMETRY_SOURCE_UART_COUNT (2U)
#define TELEMETRY_STATUS_SLOT_COUNT (5U)
#define TELEMETRY_RESULT_SUBMIT_TIMEOUT_MS (5U)
#define TELEMETRY_RESULT_POLL_TIMEOUT_MS (1U)

typedef char telemetry_msg_size_check[(sizeof(telemetry_msg_t) == 72U) ? 1 : -1];

typedef struct {
    uint8_t target;
    uint8_t cmd;
    uint8_t len;
    uint8_t valid;
    uint8_t pending;
    uint8_t payload[TELEMETRY_MSG_PAYLOAD_SIZE];
} telemetry_status_slot_t;

static uint8_t telemetry_seq = 0U;
static uint8_t telemetry_status_rr_index = 0U;
static const uart_driver_t *const telemetry_source_uart[TELEMETRY_SOURCE_UART_COUNT] = {
    &uart2_driver,
    &uart3_driver,
};
static telemetry_status_slot_t telemetry_status_slot[TELEMETRY_STATUS_SLOT_COUNT] = {
    {.target = CTRL_TARGET_SYSTEM, .cmd = CTRL_SYS_RPT_DEMO_STATUS},
    {.target = CTRL_TARGET_CHASSIS, .cmd = CTRL_CHS_RPT_STATUS},
    {.target = CTRL_TARGET_ARM, .cmd = CTRL_ARM_RPT_STATUS},
    {.target = CTRL_TARGET_LIFT, .cmd = CTRL_LIFT_RPT_STATUS},
    {.target = CTRL_TARGET_PERIPHERAL, .cmd = CTRL_PERIPH_RPT_STATUS},
};

static uint8_t telemetry_fill_msg(telemetry_msg_t *msg, uint8_t target, uint8_t cmd,
                                  const uint8_t *payload, uint8_t len)
{
    if ((msg == 0) || (len > TELEMETRY_MSG_PAYLOAD_SIZE)) {
        return 0U;
    }
    if ((len > 0U) && (payload == 0)) {
        return 0U;
    }

    memset(msg, 0, sizeof(*msg));
    msg->target = target;
    msg->cmd = cmd;
    msg->len = len;
    if (len > 0U) {
        memcpy(msg->payload, payload, len);
    }

    return 1U;
}

uint8_t telemetry_submit_result(uint8_t target, uint8_t cmd,
                                const uint8_t *payload, uint8_t len)
{
    telemetry_msg_t msg;
    osStatus_t status;

    if ((TelemetryTxQueueHandle == 0) ||
        (telemetry_fill_msg(&msg, target, cmd, payload, len) == 0U)) {
        return 0U;
    }

    status = osMessageQueuePut(TelemetryTxQueueHandle, &msg, 0U,
                               TELEMETRY_RESULT_SUBMIT_TIMEOUT_MS);

    return (status == osOK) ? 1U : 0U;
}

uint8_t telemetry_submit(uint8_t target, uint8_t cmd,
                         const uint8_t *payload, uint8_t len)
{
    return telemetry_submit_result(target, cmd, payload, len);
}

uint8_t telemetry_submit_status(uint8_t target, uint8_t cmd,
                                const uint8_t *payload, uint8_t len)
{
    uint8_t i;
    uint32_t primask;

    if ((len > TELEMETRY_MSG_PAYLOAD_SIZE) ||
        ((len > 0U) && (payload == 0))) {
        return 0U;
    }

    for (i = 0U; i < TELEMETRY_STATUS_SLOT_COUNT; ++i) {
        telemetry_status_slot_t *slot = &telemetry_status_slot[i];

        if ((slot->target == target) && (slot->cmd == cmd)) {
            primask = __get_PRIMASK();
            __disable_irq();
            slot->len = len;
            if (len > 0U) {
                memcpy(slot->payload, payload, len);
            }
            slot->valid = 1U;
            slot->pending = 1U;
            if (primask == 0U) {
                __enable_irq();
            }
            return 1U;
        }
    }

    return 0U;
}

static uint8_t telemetry_take_pending_status(telemetry_msg_t *msg)
{
    uint8_t offset;
    uint8_t found = 0U;
    uint32_t primask;

    if (msg == 0) {
        return 0U;
    }

    for (offset = 0U; offset < TELEMETRY_STATUS_SLOT_COUNT; ++offset) {
        uint8_t index = (uint8_t)((telemetry_status_rr_index + offset) %
                                  TELEMETRY_STATUS_SLOT_COUNT);
        telemetry_status_slot_t *slot = &telemetry_status_slot[index];

        primask = __get_PRIMASK();
        __disable_irq();
        if ((slot->valid != 0U) && (slot->pending != 0U)) {
            msg->target = slot->target;
            msg->cmd = slot->cmd;
            msg->len = slot->len;
            memset(msg->reserved, 0, sizeof(msg->reserved));
            if (slot->len > 0U) {
                memcpy(msg->payload, slot->payload, slot->len);
            }
            slot->pending = 0U;
            telemetry_status_rr_index = (uint8_t)((index + 1U) %
                                                  TELEMETRY_STATUS_SLOT_COUNT);
            found = 1U;
        }
        if (primask == 0U) {
            __enable_irq();
        }
        if (found != 0U) {
            return 1U;
        }
    }

    return 0U;
}

static void telemetry_send_to_all_sources(const uint8_t *frame_buf, uint16_t frame_len)
{
    uint8_t sent_mask = 0U;
    uint8_t done_mask = 0U;

    if ((frame_buf == 0) || (frame_len == 0U)) {
        return;
    }

    while (done_mask != ((1U << TELEMETRY_SOURCE_UART_COUNT) - 1U)) {
        uint8_t i;

        for (i = 0U; i < TELEMETRY_SOURCE_UART_COUNT; ++i) {
            uint8_t source_mask = (uint8_t)(1U << i);
            const uart_driver_t *uart = telemetry_source_uart[i];

            if ((sent_mask & source_mask) != 0U) {
                done_mask |= source_mask;
            } else if ((uart == 0) || (uart->send == 0)) {
                done_mask |= source_mask;
            } else if ((uart->is_tx_ready == 0) ||
                       (uart->is_tx_ready(uart->ctx) != 0U)) {
                if (uart->send(uart->ctx, frame_buf, frame_len) != 0U) {
                    sent_mask |= source_mask;
                    done_mask |= source_mask;
                }
            }
        }

        if (done_mask != ((1U << TELEMETRY_SOURCE_UART_COUNT) - 1U)) {
            osDelay(1U);
        }
    }
}

void StartTelemetryTask(void *argument)
{
    telemetry_msg_t msg;
    uint8_t frame_buf[TELEMETRY_FRAME_BUF_SIZE];

    (void)argument;

    for (;;) {
        if ((osMessageQueueGet(TelemetryTxQueueHandle, &msg, 0U,
                               TELEMETRY_RESULT_POLL_TIMEOUT_MS) == osOK) ||
            (telemetry_take_pending_status(&msg) != 0U)) {
            uint16_t frame_len;

            frame_len = control_protocol_encode_frame(CTRL_SRC_MCU,
                                                      msg.target,
                                                      msg.cmd,
                                                      telemetry_seq++,
                                                      msg.payload,
                                                      msg.len,
                                                      frame_buf,
                                                      sizeof(frame_buf));
            if (frame_len > 0U) {
                telemetry_send_to_all_sources(frame_buf, frame_len);
            }
        }
    }
}
