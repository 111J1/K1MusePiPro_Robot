#include "drv_power_voltage.h"

#include "adc.h"
#include "stm32g4xx_hal.h"

#define POWER_VOLTAGE_ADC_POLL_TIMEOUT_MS (2U)

typedef struct {
    ADC_HandleTypeDef *hadc;
} power_voltage_drv_ctx_t;

static const power_voltage_drv_ctx_t power_voltage_ctx = {
    .hadc = &hadc3,
};

static uint8_t power_voltage_drv_init(const void *ctx)
{
    const power_voltage_drv_ctx_t *voltage_ctx = (const power_voltage_drv_ctx_t *)ctx;
    ADC_HandleTypeDef *hadc;

    if ((voltage_ctx == 0) || (voltage_ctx->hadc == 0)) {
        return 0U;
    }

    hadc = voltage_ctx->hadc;
    (void)HAL_ADC_Stop(hadc);

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        return 0U;
    }

    return (HAL_ADC_Start(hadc) == HAL_OK) ? 1U : 0U;
}

static uint8_t power_voltage_drv_read_raw(const void *ctx, uint16_t *raw)
{
    const power_voltage_drv_ctx_t *voltage_ctx = (const power_voltage_drv_ctx_t *)ctx;
    uint32_t value;

    if ((voltage_ctx == 0) || (voltage_ctx->hadc == 0) || (raw == 0)) {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(voltage_ctx->hadc,
                                  POWER_VOLTAGE_ADC_POLL_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }

    value = HAL_ADC_GetValue(voltage_ctx->hadc);
    if (value > 0xFFFFUL) {
        return 0U;
    }

    *raw = (uint16_t)value;
    return 1U;
}

const power_voltage_driver_t power_voltage_driver = {
    .ctx = &power_voltage_ctx,
    .init = power_voltage_drv_init,
    .read_raw = power_voltage_drv_read_raw,
};
