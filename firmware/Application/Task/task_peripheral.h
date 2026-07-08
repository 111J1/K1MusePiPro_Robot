#ifndef _TASK_PERIPHERAL_H_
#define _TASK_PERIPHERAL_H_

#include "cmsis_os.h"
#include "mdl_peripheral.h"

typedef struct {
    uint32_t tick_ms;
    peripheral_state_t runtime;
} peripheral_state_snapshot_t;

extern volatile peripheral_state_snapshot_t g_peripheral_state;

void StartPeripheralTask(void *argument);

#endif /* _TASK_PERIPHERAL_H_ */
