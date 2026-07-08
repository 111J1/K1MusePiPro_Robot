#include "k1muse_control_core/control_protocol.h"

uint8_t k1_crc8_atm(const uint8_t *data, size_t len)
{
    uint8_t crc = 0U;
    size_t i;

    if ((data == NULL) && (len > 0U)) {
        return 0U;
    }

    for (i = 0U; i < len; ++i) {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U)
                                : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}
