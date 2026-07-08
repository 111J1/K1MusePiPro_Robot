#ifndef _TASK_CHASSIS_H
#define _TASK_CHASSIS_H

#include "cmsis_os.h"
#include "mdl_control_protocol.h"
#include <stdint.h>

typedef enum {
    CHASSIS_CMD_STOP = 0x00,
    CHASSIS_CMD_MOV = 0x01,
    CHASSIS_CMD_ODOM = 0x02,
} chassis_cmd_type_e;

typedef struct {
    uint8_t type;
    uint8_t source;
    uint8_t move_cs;
    uint8_t reserved;
    float direction;
    float v;
    float omega;
    uint32_t tick;
} chassis_cmd_msg_t;

extern osMessageQueueId_t ChassisCmdQueueHandle;

#endif // _TASK_CHASSIS_H
