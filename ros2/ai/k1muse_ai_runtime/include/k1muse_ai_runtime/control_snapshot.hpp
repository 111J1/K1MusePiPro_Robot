#pragma once

#include <cstdint>
#include <string>

namespace k1muse_ai_runtime
{

struct ControlSnapshot
{
  std::string trace_id;
  uint64_t epoch{0};
  uint8_t interaction_state{0};
  std::string interaction_state_name;
  bool wakeword_enabled{false};
  bool vision_enabled{false};
  bool vad_asr_enabled{false};
  bool tts_enabled{false};
  std::string reason;
};

}  // namespace k1muse_ai_runtime
