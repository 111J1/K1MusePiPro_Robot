#include "task_peripheral.h"

#include "dev_gas_sensor.h"
#include "dev_power_voltage.h"
#include "dev_sht31.h"
#include "drv_gas_sensor.h"
#include "drv_power_voltage.h"
#include "drv_sht31.h"
#include "mdl_control_protocol.h"
#include "mdl_peripheral_config.h"
#include "task_telemetry.h"

static gas_sensor_device_t gas_sensor = {
    .driver = &gas_sensor_driver,
};

volatile power_voltage_device_t power_voltage = {
    .adc_ref_mv = POWER_ADC_REF_MV,
    .adc_max_raw = POWER_ADC_MAX_RAW,
    .adc_sample_count = POWER_ADC_SAMPLE_COUNT,
    .voltage_scale_num = POWER_VOLTAGE_SCALE_NUM,
    .voltage_scale_den = POWER_VOLTAGE_SCALE_DEN,
    .voltage_cal_num = POWER_VOLTAGE_CAL_NUM,
    .voltage_cal_den = POWER_VOLTAGE_CAL_DEN,
    .filter_shift = POWER_VOLTAGE_FILTER_SHIFT,
    .driver = &power_voltage_driver,
};

static sht31_device_t sht31 = {
    .measure_period_ms = SHT31_MEASURE_PERIOD_MS,
    .measure_wait_ms = SHT31_MEASURE_WAIT_MS,
    .error_limit = SHT31_ERROR_LIMIT,
    .driver = &sht31_driver,
};

static peripheral_t peripheral = {
    .gas_sensor = {
        .ctx = &gas_sensor,
        .init = gas_sensor_init,
        .update = gas_sensor_update,
        .get_level = gas_sensor_get_level,
        .is_initialized = gas_sensor_is_initialized,
    },
    .power_voltage = {
        .ctx = (void *)&power_voltage,
        .init = power_voltage_init,
        .update = power_voltage_update,
        .is_initialized = power_voltage_is_initialized,
        .is_valid = power_voltage_is_valid,
        .get_power_mv = power_voltage_get_power_mv,
    },
    .sht31 = {
        .ctx = &sht31,
        .init = sht31_init,
        .update = sht31_update,
        .is_initialized = sht31_is_initialized,
        .is_valid = sht31_is_valid,
        .get_status = sht31_get_status,
        .get_temperature_centi_c = sht31_get_temperature_centi_c,
        .get_humidity_centi_pct = sht31_get_humidity_centi_pct,
    },
};

static uint32_t s_peripheral_status_phase_start_ms = 0U;
static uint32_t s_peripheral_status_last_ms = 0U;
static uint8_t s_peripheral_status_phase_ready = 0U;
volatile peripheral_state_snapshot_t g_peripheral_state;

static uint32_t task_peripheral_tick_to_ms(uint32_t tick)
{
    uint32_t tick_freq = osKernelGetTickFreq();

    if (tick_freq == 0U) {
        return tick;
    }
    return (uint32_t)(((uint64_t)tick * 1000ULL) / tick_freq);
}

static void task_peripheral_state_snapshot_update(uint32_t now_tick)
{
    const peripheral_state_t *state = peripheral_get_state(&peripheral);

    g_peripheral_state.tick_ms = task_peripheral_tick_to_ms(now_tick);
    if (state != 0) {
        g_peripheral_state.runtime = *state;
    }
}

static void task_peripheral_send_status(void)
{
    ctrl_peripheral_status_payload_t payload = {0};
    uint32_t now_ms = g_peripheral_state.tick_ms;

    if (s_peripheral_status_phase_ready == 0U) {
        if (s_peripheral_status_phase_start_ms == 0U) {
            s_peripheral_status_phase_start_ms = now_ms;
        }
        if ((uint32_t)(now_ms - s_peripheral_status_phase_start_ms) <
            PERIPHERAL_STATUS_PHASE_MS) {
            return;
        }
        s_peripheral_status_phase_ready = 1U;
        s_peripheral_status_last_ms = now_ms - PERIPHERAL_STATUS_PERIOD_MS;
    }

    if ((uint32_t)(now_ms - s_peripheral_status_last_ms) <
        PERIPHERAL_STATUS_PERIOD_MS) {
        return;
    }
    s_peripheral_status_last_ms = now_ms;

    payload.tick_ms = g_peripheral_state.tick_ms;
    payload.gas_detected = g_peripheral_state.runtime.gas_detected;
    payload.power_state = (uint8_t)g_peripheral_state.runtime.power_state;
    payload.power_mv = g_peripheral_state.runtime.power_mv;
    payload.temperature_centi_c = g_peripheral_state.runtime.temperature_centi_c;
    payload.humidity_centi_pct = g_peripheral_state.runtime.humidity_centi_pct;

    (void)telemetry_submit_status(CTRL_TARGET_PERIPHERAL,
                                  CTRL_PERIPH_RPT_STATUS,
                                  (const uint8_t *)&payload,
                                  (uint8_t)sizeof(payload));
}

void StartPeripheralTask(void *argument)
{
    uint32_t now_tick;

    (void)argument;

    peripheral_init(&peripheral);

    for (;;) {
        now_tick = osKernelGetTickCount();

        peripheral_update(&peripheral, PERIPHERAL_TASK_PERIOD_MS);
        task_peripheral_state_snapshot_update(now_tick);
        task_peripheral_send_status();

        osDelayUntil(now_tick + PERIPHERAL_TASK_PERIOD_MS);
    }
}
