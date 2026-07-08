#ifndef _TASK_COMMAND_H
#define _TASK_COMMAND_H

#include "cmsis_os.h"
#include <stdint.h>

typedef struct {
    uint32_t host_last_valid_tick;
    uint32_t bt_last_valid_tick;
} command_uart_state_t;

extern volatile command_uart_state_t g_command_uart_state;

void StartCommandTask(void *argument);

#endif // TASK_COMMAND_H
