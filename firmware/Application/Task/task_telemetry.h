#ifndef _TASK_TELEMETRY_H_
#define _TASK_TELEMETRY_H_

#include "cmsis_os.h"
#include "mdl_control_protocol.h"
#include <stdint.h>

#define TELEMETRY_MSG_PAYLOAD_SIZE CTRL_PROTOCOL_MAX_PAYLOAD

typedef struct {
    uint8_t target;
    uint8_t cmd;
    uint8_t len;
    uint8_t reserved[5];
    uint8_t payload[TELEMETRY_MSG_PAYLOAD_SIZE];
} telemetry_msg_t;

extern osMessageQueueId_t TelemetryTxQueueHandle;

void StartTelemetryTask(void *argument);
uint8_t telemetry_submit(uint8_t target, uint8_t cmd,
                         const uint8_t *payload, uint8_t len);
uint8_t telemetry_submit_result(uint8_t target, uint8_t cmd,
                                const uint8_t *payload, uint8_t len);
uint8_t telemetry_submit_status(uint8_t target, uint8_t cmd,
                                const uint8_t *payload, uint8_t len);

#endif /* _TASK_TELEMETRY_H_ */
