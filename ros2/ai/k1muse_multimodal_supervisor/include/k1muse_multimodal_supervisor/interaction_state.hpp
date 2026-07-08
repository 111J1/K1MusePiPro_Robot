#pragma once

#include <cstdint>

namespace k1muse_multimodal_supervisor {

enum class State : uint8_t {
  BOOT = 0,
  IDLE = 1,
  WAKE_ACK = 2,
  LISTENING = 3,
  INTENT_PROCESSING = 4,
  TTS_RUNNING = 5,
  TARGETING = 6,
  EMERGENCY_OR_FAULT = 7,
};

/// Return a human-readable name for the given state.
const char* state_name(State s);

}  // namespace k1muse_multimodal_supervisor
