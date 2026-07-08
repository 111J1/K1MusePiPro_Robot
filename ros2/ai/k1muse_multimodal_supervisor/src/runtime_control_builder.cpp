#include "k1muse_multimodal_supervisor/runtime_control_builder.hpp"

namespace k1muse_multimodal_supervisor {

// State -> flags mapping (see task spec table):
//   State               wakeword  vision  vad_asr  tts
//   BOOT                OFF       OFF     OFF      OFF
//   IDLE                ON        ON      OFF      OFF
//   WAKE_ACK            OFF       OFF     OFF      OFF
//   LISTENING           OFF       OFF     ON       OFF
//   INTENT_PROCESSING   OFF       OFF     OFF      OFF
//   TTS_RUNNING         OFF       OFF     OFF      ON
//   TARGETING           ON        ON      OFF      OFF
//   EMERGENCY_OR_FAULT  OFF       OFF     OFF      OFF

ControlFlags RuntimeControlBuilder::flags_for_state(State state) {
  ControlFlags f;
  switch (state) {
    case State::IDLE:
    case State::TARGETING:
      f.wakeword_enabled = true;
      f.vision_enabled   = true;
      break;

    case State::LISTENING:
      f.vad_asr_enabled = true;
      break;

    case State::TTS_RUNNING:
      f.tts_enabled = true;
      break;

    // All other states: everything OFF (defaults).
    case State::BOOT:
    case State::WAKE_ACK:
    case State::INTENT_PROCESSING:
    case State::EMERGENCY_OR_FAULT:
    default:
      break;
  }
  return f;
}

}  // namespace k1muse_multimodal_supervisor
