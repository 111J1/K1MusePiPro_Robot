#pragma once

#include "k1muse_multimodal_supervisor/interaction_state.hpp"

namespace k1muse_multimodal_supervisor {

/// Flags that map 1:1 to AiRuntimeControl message fields.
struct ControlFlags {
  bool wakeword_enabled = false;
  bool vision_enabled = false;
  bool vad_asr_enabled = false;
  bool tts_enabled = false;
};

/// Pure mapping from interaction state to runtime-control flags.
class RuntimeControlBuilder {
public:
  /// Get the control flags for a given state.
  static ControlFlags flags_for_state(State state);
};

}  // namespace k1muse_multimodal_supervisor
