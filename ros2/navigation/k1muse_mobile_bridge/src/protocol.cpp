#include "k1muse_mobile_bridge/protocol.hpp"

#include <cstring>
#include <stdexcept>
#include <zlib.h>

namespace k1muse_mobile_bridge
{
namespace
{
constexpr uint8_t kMagic[4] = {'K', '1', 'M', 'B'};
constexpr uint8_t kVersion = 1;
constexpr size_t kMaxPayloadSize = 4U * 1024U * 1024U;
}  // namespace

void append_u8(std::vector<uint8_t> & out, uint8_t value)
{
  out.push_back(value);
}

void append_u16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32(std::vector<uint8_t> & out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 16U) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 24U) & 0xFFU));
}

void append_i32(std::vector<uint8_t> & out, int32_t value)
{
  append_u32(out, static_cast<uint32_t>(value));
}

void append_float(std::vector<uint8_t> & out, float value)
{
  uint32_t raw = 0;
  static_assert(sizeof(raw) == sizeof(value), "float must be 32-bit");
  std::memcpy(&raw, &value, sizeof(raw));
  append_u32(out, raw);
}

void append_string(std::vector<uint8_t> & out, const std::string & value)
{
  append_u16(out, static_cast<uint16_t>(value.size()));
  out.insert(out.end(), value.begin(), value.end());
}

std::vector<uint8_t> encode_frame(
  MessageType type,
  uint32_t seq,
  const std::vector<uint8_t> & payload,
  uint16_t flags)
{
  if (payload.size() > kMaxPayloadSize) {
    throw std::runtime_error("mobile bridge payload too large");
  }

  std::vector<uint8_t> frame;
  frame.reserve(16U + payload.size());
  frame.insert(frame.end(), std::begin(kMagic), std::end(kMagic));
  append_u8(frame, kVersion);
  append_u8(frame, static_cast<uint8_t>(type));
  append_u16(frame, flags);
  append_u32(frame, seq);
  append_u32(frame, static_cast<uint32_t>(payload.size()));
  append_u32(frame, crc32(0L, payload.data(), static_cast<uInt>(payload.size())));
  frame.insert(frame.end(), payload.begin(), payload.end());
  return frame;
}

}  // namespace k1muse_mobile_bridge
