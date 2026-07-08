#ifndef K1MUSE_MOBILE_BRIDGE_PROTOCOL_HPP_
#define K1MUSE_MOBILE_BRIDGE_PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_mobile_bridge
{

enum class MessageType : uint8_t
{
  HELLO = 0x01,
  HEARTBEAT = 0x02,
  MAP_INFO = 0x10,
  MAP_TILE = 0x11,
  ROBOT_POSE = 0x20,
  TELEOP_CMD = 0x30,
  STOP = 0x31,
  NAV_GOAL = 0x40,
  NAV_CANCEL = 0x41,
  NAV_STATUS = 0x42,
  NAV_PATH = 0x43,
  BRIDGE_CONTROL = 0x48,
  BRIDGE_STATUS = 0x49,
  MAP_CONTROL = 0x50,
  MAP_CONTROL_STATUS = 0x51,
  MAP_LIBRARY_REQUEST = 0x60,
  MAP_LIBRARY_STATUS = 0x61,
  MAP_LIBRARY_LIST = 0x62,
  MAP_REGIONS_DATA = 0x63,
  ERROR = 0x7F,
};

enum class TileEncoding : uint8_t
{
  RAW = 0,
  ZLIB = 1,
};

std::vector<uint8_t> encode_frame(
  MessageType type,
  uint32_t seq,
  const std::vector<uint8_t> & payload,
  uint16_t flags = 0);

void append_u8(std::vector<uint8_t> & out, uint8_t value);
void append_u16(std::vector<uint8_t> & out, uint16_t value);
void append_u32(std::vector<uint8_t> & out, uint32_t value);
void append_i32(std::vector<uint8_t> & out, int32_t value);
void append_float(std::vector<uint8_t> & out, float value);
void append_string(std::vector<uint8_t> & out, const std::string & value);

}  // namespace k1muse_mobile_bridge

#endif  // K1MUSE_MOBILE_BRIDGE_PROTOCOL_HPP_
