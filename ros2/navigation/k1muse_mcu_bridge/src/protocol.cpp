#include "k1muse_mcu_bridge/protocol.hpp"

namespace k1muse_mcu_bridge {

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

std::vector<uint8_t> build_frame(Target target, uint8_t cmd, uint8_t seq,
                                 const uint8_t* payload, uint8_t len) {
    std::vector<uint8_t> frame;
    frame.reserve(HEADER_SIZE + len + 1);

    frame.push_back(SOF1);
    frame.push_back(SOF2);
    frame.push_back(static_cast<uint8_t>(Src::HOST));
    frame.push_back(static_cast<uint8_t>(target));
    frame.push_back(cmd);
    frame.push_back(seq);
    frame.push_back(len);

    if (payload && len > 0)
        frame.insert(frame.end(), payload, payload + len);

    uint8_t c = crc8(frame.data() + 2, frame.size() - 2);
    frame.push_back(c);

    return frame;
}

}  // namespace k1muse_mcu_bridge
