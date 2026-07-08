#pragma once

#include <cstdint>

namespace k1muse_multimodal_supervisor {

enum class Event : uint8_t {
  // Readiness
  RUNTIME_READY_CHANGED,
  AUDIO_READY_CHANGED,
  INTENT_READY_CHANGED,

  // Voice flow
  WAKEWORD_ACCEPTED,
  WAKE_ACK_PLAYBACK_DONE,
  WAKE_ACK_PLAYBACK_FAILED,
  LISTEN_RESULT_RECEIVED,
  LISTEN_FAILED,
  LISTEN_CANCELLED,
  NO_SPEECH_TIMEOUT,

  // Intent
  INTENT_FINISHED,
  INTENT_FAILED,

  // TTS
  TTS_DONE,
  TTS_FAILED,
  TTS_PLAYBACK_DONE,
  TTS_PLAYBACK_FAILED,

  // Vision/targeting
  TARGET_REQUEST_RECEIVED,
  TARGET_RESPONSE_SENT,

  // System
  SYSTEM_FAULT,
  FAULT_CLEARED,
};

}  // namespace k1muse_multimodal_supervisor
