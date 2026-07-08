#ifndef _MDL_CONTROL_ARBITRATION_H_
#define _MDL_CONTROL_ARBITRATION_H_

#include "mdl_control_protocol.h"
#include <stdint.h>

typedef struct {
    control_source_e active_source;
    uint32_t last_control_tick;
} module_control_arbitration_t;

void control_arbitration_init(module_control_arbitration_t *arbitration);
uint8_t control_arbitration_can_accept(const module_control_arbitration_t *arbitration,
                                       control_source_e source);
void control_arbitration_accept(module_control_arbitration_t *arbitration,
                                control_source_e source, uint32_t tick);
void control_arbitration_release(module_control_arbitration_t *arbitration);
uint8_t control_arbitration_is_timeout(const module_control_arbitration_t *arbitration,
                                       uint32_t now_tick, uint32_t timeout_ms);

#endif /* _MDL_CONTROL_ARBITRATION_H_ */
