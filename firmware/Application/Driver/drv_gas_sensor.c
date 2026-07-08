#include "drv_gas_sensor.h"

#include "main.h"
#include "stm32g4xx_hal.h"

typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
} gas_sensor_drv_ctx_t;

static const gas_sensor_drv_ctx_t gas_sensor_ctx = {
    .gpio_port = GAS_SENSOR_GPIO_Port,
    .gpio_pin = GAS_SENSOR_Pin,
};

static uint8_t gas_sensor_drv_init(const void *ctx)
{
    const gas_sensor_drv_ctx_t *sensor_ctx = (const gas_sensor_drv_ctx_t *)ctx;

    return ((sensor_ctx != 0) && (sensor_ctx->gpio_port != 0)) ? 1U : 0U;
}

static uint8_t gas_sensor_drv_get_level(const void *ctx)
{
    const gas_sensor_drv_ctx_t *sensor_ctx = (const gas_sensor_drv_ctx_t *)ctx;

    if ((sensor_ctx == 0) || (sensor_ctx->gpio_port == 0)) {
        return 0U;
    }

    return (HAL_GPIO_ReadPin(sensor_ctx->gpio_port, sensor_ctx->gpio_pin) == GPIO_PIN_SET) ? 1U : 0U;
}

const gas_sensor_driver_t gas_sensor_driver = {
    .ctx = &gas_sensor_ctx,
    .init = gas_sensor_drv_init,
    .get_level = gas_sensor_drv_get_level,
};
