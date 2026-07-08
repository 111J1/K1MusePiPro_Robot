#include "drv_lift_home_sensor.h"

#include "main.h"
#include "stm32g4xx_hal.h"

typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
} lift_home_sensor_drv_ctx_t;

static const lift_home_sensor_drv_ctx_t lift_home_sensor_ctx[LIFT_HOME_SENSOR_DRV_COUNT] = {
    [LIFT_HOME_SENSOR_DRV_BOTTOM] = {
        .gpio_port = LIFT_BOTTOM_SENSOR_GPIO_Port,
        .gpio_pin = LIFT_BOTTOM_SENSOR_Pin,
    },
    [LIFT_HOME_SENSOR_DRV_MIDDLE] = {
        .gpio_port = LIFT_MIDDLE_SENSOR_GPIO_Port,
        .gpio_pin = LIFT_MIDDLE_SENSOR_Pin,
    },
    [LIFT_HOME_SENSOR_DRV_TOP] = {
        .gpio_port = LIFT_TOP_SENSOR_GPIO_Port,
        .gpio_pin = LIFT_TOP_SENSOR_Pin,
    },
};

static uint8_t lift_home_sensor_drv_init(const void *ctx)
{
    const lift_home_sensor_drv_ctx_t *sensor_ctx = (const lift_home_sensor_drv_ctx_t *)ctx;

    return ((sensor_ctx != 0) && (sensor_ctx->gpio_port != 0)) ? 1U : 0U;
}

static uint8_t lift_home_sensor_drv_get_level(const void *ctx)
{
    const lift_home_sensor_drv_ctx_t *sensor_ctx = (const lift_home_sensor_drv_ctx_t *)ctx;

    if ((sensor_ctx == 0) || (sensor_ctx->gpio_port == 0)) {
        return 0U;
    }

    return (HAL_GPIO_ReadPin(sensor_ctx->gpio_port, sensor_ctx->gpio_pin) == GPIO_PIN_SET) ? 1U : 0U;
}

#define LIFT_HOME_SENSOR_DRIVER(_id)     \
    {                                    \
        .ctx = &lift_home_sensor_ctx[_id], \
        .init = lift_home_sensor_drv_init, \
        .get_level = lift_home_sensor_drv_get_level}

const lift_home_sensor_driver_t lift_bottom_sensor_driver =
    LIFT_HOME_SENSOR_DRIVER(LIFT_HOME_SENSOR_DRV_BOTTOM);

const lift_home_sensor_driver_t lift_middle_sensor_driver =
    LIFT_HOME_SENSOR_DRIVER(LIFT_HOME_SENSOR_DRV_MIDDLE);

const lift_home_sensor_driver_t lift_top_sensor_driver =
    LIFT_HOME_SENSOR_DRIVER(LIFT_HOME_SENSOR_DRV_TOP);

const lift_home_sensor_driver_t lift_home_sensor_driver =
    LIFT_HOME_SENSOR_DRIVER(LIFT_HOME_SENSOR_DRV_BOTTOM);

const lift_home_sensor_driver_t *lift_home_sensor_drv_get(lift_home_sensor_drv_id_e id)
{
    static const lift_home_sensor_driver_t *const drivers[LIFT_HOME_SENSOR_DRV_COUNT] = {
        [LIFT_HOME_SENSOR_DRV_BOTTOM] = &lift_bottom_sensor_driver,
        [LIFT_HOME_SENSOR_DRV_MIDDLE] = &lift_middle_sensor_driver,
        [LIFT_HOME_SENSOR_DRV_TOP] = &lift_top_sensor_driver,
    };

    if (id >= LIFT_HOME_SENSOR_DRV_COUNT) {
        return 0;
    }

    return drivers[id];
}
