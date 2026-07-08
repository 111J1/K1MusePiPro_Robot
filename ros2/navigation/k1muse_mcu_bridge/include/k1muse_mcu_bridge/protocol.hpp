#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>

namespace k1muse_mcu_bridge {

// ---- frame constants -------------------------------------------------------
constexpr uint8_t  SOF1         = 0xA5;
constexpr uint8_t  SOF2         = 0x5A;
constexpr uint8_t  MAX_PAYLOAD  = 64;
constexpr uint8_t  HEADER_SIZE  = 7;
constexpr uint8_t  MAX_FRAME    = HEADER_SIZE + MAX_PAYLOAD + 1;  // 72

// ---- source identifiers ----------------------------------------------------
enum class Src : uint8_t {
    NONE = 0x00,
    BT   = 0x01,
    HOST = 0x02,
    MCU  = 0x10,
};

// ---- target modules (all defined; arm/lift ready for extension) ------------
enum class Target : uint8_t {
    SYSTEM  = 0x00,
    CHASSIS = 0x01,
    ARM     = 0x02,
    LIFT    = 0x03,
};

// ---- chassis commands ------------------------------------------------------
enum class ChassisCmd : uint8_t {
    STOP = 0x00,
    MOV  = 0x01,
    ODOM = 0x02,
};

constexpr uint8_t STATUS_CMD = 0x80;

// ---- CRC-8/ATM -------------------------------------------------------------
uint8_t crc8(const uint8_t* data, size_t len);

// ---- serialisation helpers -------------------------------------------------
inline void write_le(uint8_t* buf, uint32_t v) {
    buf[0] = v & 0xFF; buf[1] = (v >> 8) & 0xFF;
    buf[2] = (v >> 16) & 0xFF; buf[3] = (v >> 24) & 0xFF;
}
inline void write_le(uint8_t* buf, float v) {
    uint32_t raw; __builtin_memcpy(&raw, &v, 4); write_le(buf, raw);
}
inline uint32_t read_le_u32(const uint8_t* buf) {
    return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
}
inline float read_le_f32(const uint8_t* buf) {
    uint32_t raw = read_le_u32(buf);
    float v; __builtin_memcpy(&v, &raw, 4); return v;
}

// ---- payload sizes (compile-time checks) -----------------------------------
constexpr uint8_t CHASSIS_STATUS_PAYLOAD_SIZE = 32;
constexpr uint8_t CHASSIS_MOV_PAYLOAD_SIZE    = 13;
constexpr uint8_t CHASSIS_ODOM_PAYLOAD_SIZE   = 12;

// ---- frame building --------------------------------------------------------
std::vector<uint8_t> build_frame(Target target, uint8_t cmd, uint8_t seq,
                                 const uint8_t* payload, uint8_t len);

}  // namespace k1muse_mcu_bridge
