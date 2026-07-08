#ifndef TASK_DEMO_H
#define TASK_DEMO_H

#include "cmsis_os.h"
#include <stdint.h>

typedef enum {
    DEMO_ID_NONE = 0x00,
    DEMO_ID_STATIC_PICK_PLACE = 0x01,
    DEMO_ID_LAYER_PICK = 0x02,
    DEMO_ID_LAYER_PLACE = 0x03,
    DEMO_ID_LAYER_TRANSFER = 0x04,
    DEMO_ID_KEY_TURN = 0x05,
    DEMO_ID_CABINET_PULL = 0x06,
} demo_id_e;

typedef enum {
    DEMO_CMD_STOP = 0x00,
    DEMO_CMD_RUN = 0x01,
    DEMO_CMD_HOME = 0x02,
} demo_cmd_type_e;

typedef enum {
    DEMO_STATE_IDLE = 0,
    DEMO_STATE_RUNNING,
    DEMO_STATE_DONE,
    DEMO_STATE_ABORT,
    DEMO_STATE_FAULT,
} demo_state_e;

typedef enum {
    DEMO_FAULT_NONE = 0,
    DEMO_FAULT_UNKNOWN_DEMO,
    DEMO_FAULT_BAD_LAYER,
    DEMO_FAULT_BAD_VARIANT,
    DEMO_FAULT_QUEUE_FULL,
    DEMO_FAULT_ARM,
    DEMO_FAULT_LIFT,
    DEMO_FAULT_TIMEOUT,
} demo_fault_e;

typedef struct {
    uint8_t type;
    uint8_t source;
    uint8_t seq;
    uint8_t demo_id;
    uint8_t src_layer;
    uint8_t dst_layer;
    uint8_t variant;
    uint8_t reserved;
    uint32_t tick;
} demo_cmd_msg_t;

typedef struct {
    uint32_t tick_ms;
    demo_state_e state;
    demo_fault_e fault;
    uint8_t demo_id;
    uint8_t step_index;
    uint8_t step_count;
    uint8_t active;
} demo_state_snapshot_t;

extern osMessageQueueId_t DemoCmdQueueHandle;
extern volatile demo_state_snapshot_t g_demo_state;

void StartDemoTask(void *argument);

#endif /* TASK_DEMO_H */
