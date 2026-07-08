#include "drv_sht31.h"

#include "i2c.h"
#include "stm32g4xx_hal.h"

#define SHT31_I2C_ADDRESS (0x44U << 1)
#define SHT31_I2C_TIMEOUT_MS (5U)
#define SHT31_CMD_SOFT_RESET (0x30A2U)
#define SHT31_CMD_MEASURE_MEDIUM_NO_STRETCH (0x240BU)
#define SHT31_CRC_INIT (0xFFU)
#define SHT31_CRC_POLYNOMIAL (0x31U)

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    uint32_t timeout_ms;
} sht31_drv_ctx_t;

static const sht31_drv_ctx_t sht31_ctx = {
    .hi2c = &hi2c4,
    .address = SHT31_I2C_ADDRESS,
    .timeout_ms = SHT31_I2C_TIMEOUT_MS,
};

static uint8_t sht31_drv_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = SHT31_CRC_INIT;
    uint8_t i;

    while (len > 0U) {
        crc ^= *data;
        data++;
        len--;

        for (i = 0U; i < 8U; i++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ SHT31_CRC_POLYNOMIAL);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

static uint8_t sht31_drv_send_command(const sht31_drv_ctx_t *sensor_ctx,
                                      uint16_t command)
{
    uint8_t buffer[2];

    if ((sensor_ctx == 0) || (sensor_ctx->hi2c == 0)) {
        return 0U;
    }

    buffer[0] = (uint8_t)(command >> 8);
    buffer[1] = (uint8_t)(command & 0xFFU);

    return (HAL_I2C_Master_Transmit(sensor_ctx->hi2c,
                                    sensor_ctx->address,
                                    buffer,
                                    (uint16_t)sizeof(buffer),
                                    sensor_ctx->timeout_ms) == HAL_OK)
               ? 1U
               : 0U;
}

static uint8_t sht31_drv_init(const void *ctx)
{
    const sht31_drv_ctx_t *sensor_ctx = (const sht31_drv_ctx_t *)ctx;

    if ((sensor_ctx == 0) || (sensor_ctx->hi2c == 0)) {
        return 0U;
    }

    if (HAL_I2C_IsDeviceReady(sensor_ctx->hi2c,
                              sensor_ctx->address,
                              2U,
                              sensor_ctx->timeout_ms) != HAL_OK) {
        return 0U;
    }

    if (sht31_drv_send_command(sensor_ctx, SHT31_CMD_SOFT_RESET) == 0U) {
        return 0U;
    }
    HAL_Delay(2U);

    return (HAL_I2C_IsDeviceReady(sensor_ctx->hi2c,
                                  sensor_ctx->address,
                                  2U,
                                  sensor_ctx->timeout_ms) == HAL_OK)
               ? 1U
               : 0U;
}

static uint8_t sht31_drv_start_measurement(const void *ctx)
{
    return sht31_drv_send_command((const sht31_drv_ctx_t *)ctx,
                                  SHT31_CMD_MEASURE_MEDIUM_NO_STRETCH);
}

static uint8_t sht31_drv_read_measurement(const void *ctx, sht31_raw_data_t *data)
{
    const sht31_drv_ctx_t *sensor_ctx = (const sht31_drv_ctx_t *)ctx;
    uint8_t buffer[6];

    if ((sensor_ctx == 0) || (sensor_ctx->hi2c == 0) || (data == 0)) {
        return 0U;
    }

    if (HAL_I2C_Master_Receive(sensor_ctx->hi2c,
                               sensor_ctx->address,
                               buffer,
                               (uint16_t)sizeof(buffer),
                               sensor_ctx->timeout_ms) != HAL_OK) {
        return 0U;
    }

    if ((sht31_drv_crc8(&buffer[0], 2U) != buffer[2]) ||
        (sht31_drv_crc8(&buffer[3], 2U) != buffer[5])) {
        return 0U;
    }

    data->temperature_raw = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    data->humidity_raw = (uint16_t)(((uint16_t)buffer[3] << 8) | buffer[4]);

    return 1U;
}

const sht31_driver_t sht31_driver = {
    .ctx = &sht31_ctx,
    .init = sht31_drv_init,
    .start_measurement = sht31_drv_start_measurement,
    .read_measurement = sht31_drv_read_measurement,
};
