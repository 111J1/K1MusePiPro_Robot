#include "mdl_control_arbitration.h"

void control_arbitration_init(module_control_arbitration_t *arbitration)
{
    if (arbitration != 0) {
        arbitration->active_source = CTRL_SRC_NONE;
        arbitration->last_control_tick = 0U;
    }
}

uint8_t control_arbitration_can_accept(const module_control_arbitration_t *arbitration,
                                       control_source_e source)
{
    if ((arbitration == 0) || (source == CTRL_SRC_NONE)) {
        return 0U;
    }

    /* A module can be idle or owned by the same source. Other sources wait. */
    return ((arbitration->active_source == CTRL_SRC_NONE) ||
            (arbitration->active_source == source))
               ? 1U
               : 0U;
}

void control_arbitration_accept(module_control_arbitration_t *arbitration,
                                control_source_e source, uint32_t tick)
{
    if ((arbitration != 0) && (source != CTRL_SRC_NONE)) {
        /* Accepting a motion command refreshes ownership timeout. */
        arbitration->active_source = source;
        arbitration->last_control_tick = tick;
    }
}

void control_arbitration_release(module_control_arbitration_t *arbitration)
{
    if (arbitration != 0) {
        arbitration->active_source = CTRL_SRC_NONE;
        arbitration->last_control_tick = 0U;
    }
}

uint8_t control_arbitration_is_timeout(const module_control_arbitration_t *arbitration,
                                       uint32_t now_tick, uint32_t timeout_ms)
{
    if ((arbitration == 0) || (arbitration->active_source == CTRL_SRC_NONE)) {
        return 0U;
    }

    /* Unsigned subtraction keeps working across RTOS tick wraparound. */
    return ((now_tick - arbitration->last_control_tick) > timeout_ms) ? 1U : 0U;
}
