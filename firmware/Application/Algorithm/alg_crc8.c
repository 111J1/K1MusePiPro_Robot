#include "alg_crc8.h"

uint8_t alg_crc8_atm(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;

    if (data == 0) {
        return 0U;
    }

    while (len-- > 0U) {
        crc ^= *data++;
        for (uint8_t i = 0U; i < 8U; ++i) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}
