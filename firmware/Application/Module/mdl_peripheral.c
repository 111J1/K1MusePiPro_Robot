#include "mdl_peripheral.h"

#include "mdl_peripheral_config.h"

static peripheral_power_state_e peripheral_get_power_state(uint8_t valid,
                                                           uint16_t power_mv)
{
    if (valid == 0U) {
        return PERIPHERAL_POWER_FAULT;
    }
    if (power_mv < POWER_FAULT_THRESHOLD_MV) {
        return PERIPHERAL_POWER_FAULT;
    }
    if (power_mv < POWER_CRITICAL_THRESHOLD_MV) {
        return PERIPHERAL_POWER_CRITICAL;
    }
    if (power_mv < POWER_LOW_THRESHOLD_MV) {
        return PERIPHERAL_POWER_LOW;
    }

    return PERIPHERAL_POWER_NORMAL;
}

static peripheral_power_state_e peripheral_confirm_power_state(peripheral_t *peripheral,
                                                               peripheral_power_state_e measured_state,
                                                               uint32_t dt_ms)
{
    if (measured_state == PERIPHERAL_POWER_NORMAL) {
        peripheral->power_pending_state = PERIPHERAL_POWER_NORMAL;
        peripheral->power_pending_elapsed_ms = 0U;
        peripheral->state.power_pending_state = peripheral->power_pending_state;
        peripheral->state.power_pending_elapsed_ms = peripheral->power_pending_elapsed_ms;
        return PERIPHERAL_POWER_NORMAL;
    }

    if (measured_state != peripheral->power_pending_state) {
        peripheral->power_pending_state = measured_state;
        peripheral->power_pending_elapsed_ms = 0U;
    } else if (peripheral->power_pending_elapsed_ms < POWER_STATE_CONFIRM_MS) {
        peripheral->power_pending_elapsed_ms += dt_ms;
        if (peripheral->power_pending_elapsed_ms > POWER_STATE_CONFIRM_MS) {
            peripheral->power_pending_elapsed_ms = POWER_STATE_CONFIRM_MS;
        }
    }

    peripheral->state.power_pending_state = peripheral->power_pending_state;
    peripheral->state.power_pending_elapsed_ms = peripheral->power_pending_elapsed_ms;

    return (peripheral->power_pending_elapsed_ms >= POWER_STATE_CONFIRM_MS)
               ? measured_state
               : peripheral->state.power_state;
}

void peripheral_init(peripheral_t *peripheral)
{
    if (peripheral == 0) {
        return;
    }

    peripheral->state.gas_detected = 0U;
    peripheral->state.gas_level = 0U;
    peripheral->state.gas_initialized = 0U;
    peripheral->state.power_initialized = 0U;
    peripheral->state.power_valid = 0U;
    peripheral->state.sht31_initialized = 0U;
    peripheral->state.sht31_valid = 0U;
    peripheral->state.sht31_status = PERIPHERAL_SHT31_STATUS_ERR;
    peripheral->state.power_state = PERIPHERAL_POWER_NORMAL;
    peripheral->state.power_pending_state = PERIPHERAL_POWER_NORMAL;
    peripheral->state.power_pending_elapsed_ms = 0U;
    peripheral->state.power_mv = 0U;
    peripheral->state.temperature_centi_c = 0;
    peripheral->state.humidity_centi_pct = 0U;
    peripheral->power_pending_state = PERIPHERAL_POWER_NORMAL;
    peripheral->power_pending_elapsed_ms = 0U;

    if (peripheral->gas_sensor.init != 0) {
        peripheral->gas_sensor.init(peripheral->gas_sensor.ctx);
    }
    if (peripheral->power_voltage.init != 0) {
        peripheral->power_voltage.init(peripheral->power_voltage.ctx);
    }
    if (peripheral->sht31.init != 0) {
        peripheral->sht31.init(peripheral->sht31.ctx);
    }
}

void peripheral_update(peripheral_t *peripheral, uint32_t dt_ms)
{
    uint8_t gas_level = 0U;
    uint8_t power_valid = 0U;
    uint8_t sht31_valid = 0U;
    uint16_t power_mv = 0U;
    int16_t temperature_centi_c = 0;
    uint16_t humidity_centi_pct = 0U;
    peripheral_power_state_e measured_power_state;

    if (peripheral == 0) {
        return;
    }

    if (peripheral->gas_sensor.update != 0) {
        peripheral->gas_sensor.update(peripheral->gas_sensor.ctx);
    }
    if (peripheral->power_voltage.update != 0) {
        peripheral->power_voltage.update(peripheral->power_voltage.ctx);
    }
    if (peripheral->sht31.update != 0) {
        peripheral->sht31.update(peripheral->sht31.ctx, dt_ms);
    }

    if (peripheral->gas_sensor.is_initialized != 0) {
        peripheral->state.gas_initialized =
            peripheral->gas_sensor.is_initialized(peripheral->gas_sensor.ctx);
    }
    if (peripheral->gas_sensor.get_level != 0) {
        gas_level = peripheral->gas_sensor.get_level(peripheral->gas_sensor.ctx);
    }

    if (peripheral->power_voltage.is_initialized != 0) {
        peripheral->state.power_initialized =
            peripheral->power_voltage.is_initialized(peripheral->power_voltage.ctx);
    }
    if (peripheral->power_voltage.is_valid != 0) {
        power_valid = peripheral->power_voltage.is_valid(peripheral->power_voltage.ctx);
    }
    if (peripheral->power_voltage.get_power_mv != 0) {
        power_mv = peripheral->power_voltage.get_power_mv(peripheral->power_voltage.ctx);
    }

    if (peripheral->sht31.is_initialized != 0) {
        peripheral->state.sht31_initialized =
            peripheral->sht31.is_initialized(peripheral->sht31.ctx);
    }
    if (peripheral->sht31.is_valid != 0) {
        sht31_valid = peripheral->sht31.is_valid(peripheral->sht31.ctx);
    }
    if (peripheral->sht31.get_status != 0) {
        peripheral->state.sht31_status =
            (peripheral_sht31_status_e)peripheral->sht31.get_status(peripheral->sht31.ctx);
    }
    if (peripheral->sht31.get_temperature_centi_c != 0) {
        temperature_centi_c =
            peripheral->sht31.get_temperature_centi_c(peripheral->sht31.ctx);
    }
    if (peripheral->sht31.get_humidity_centi_pct != 0) {
        humidity_centi_pct =
            peripheral->sht31.get_humidity_centi_pct(peripheral->sht31.ctx);
    }

    peripheral->state.gas_level = gas_level;
    peripheral->state.gas_detected =
        ((peripheral->state.gas_initialized != 0U) &&
         (gas_level == GAS_SENSOR_DETECTED_LEVEL))
            ? 1U
            : 0U;
    peripheral->state.power_valid = power_valid;
    peripheral->state.power_mv = power_mv;
    peripheral->state.sht31_valid = sht31_valid;
    peripheral->state.temperature_centi_c = temperature_centi_c;
    peripheral->state.humidity_centi_pct = humidity_centi_pct;
    measured_power_state = peripheral_get_power_state(power_valid, power_mv);
    peripheral->state.power_state =
        peripheral_confirm_power_state(peripheral, measured_power_state, dt_ms);
}

const peripheral_state_t *peripheral_get_state(const peripheral_t *peripheral)
{
    return (peripheral != 0) ? &peripheral->state : 0;
}
