#pragma once

#include <cstdint>
#include <string>

#include "k1muse_multimodal_supervisor/interaction_event.hpp"
#include "k1muse_multimodal_supervisor/interaction_state.hpp"

namespace k1muse_multimodal_supervisor {

/// Context carried with every event, used for stale-event rejection
/// and conditional transitions.
struct TransitionContext {
  // Readiness
  bool runtime_ready = false;
  bool audio_ready = false;
  bool intent_ready = false;

  // Active interaction identity (for stale rejection)
  std::string active_trace_id;
  uint64_t active_epoch = 0;
  std::string active_request_id;
  std::string active_utterance_id;

  // Event-specific identity
  std::string event_trace_id;
  uint64_t event_epoch = 0;
  std::string event_request_id;
  std::string event_utterance_id;

  // Conditional flags
  bool has_fallback_tts = false;
  bool has_pending_tts = true;  // false → skip TTS_RUNNING, go directly to IDLE
};

/// Result of a state-machine transition.
struct TransitionResult {
  State new_state;
  bool accepted;        // false if the event was stale / rejected
  std::string reason;   // human-readable explanation
};

/// Pure state machine for the K1MUSE multimodal interaction lifecycle.
/// No ROS dependency.
class InteractionStateMachine {
public:
  InteractionStateMachine();

  /// Current state.
  State current_state() const;

  /// Force-set state (e.g. for initialisation or recovery).
  void set_state(State state);

  /// Process an event with context.  Returns the transition result.
  TransitionResult process(Event event, const TransitionContext& ctx);

  /// Update readiness flags without triggering a state transition.
  /// Returns true if all three readiness flags are now true and the
  /// machine is still in BOOT (caller should call process with a
  /// readiness event to trigger BOOT -> IDLE).
  bool update_readiness(bool runtime_ready, bool audio_ready, bool intent_ready);

private:
  State state_;
  bool runtime_ready_ = false;
  bool audio_ready_ = false;
  bool intent_ready_ = false;

  // Tracking for the two-event TTS -> IDLE transition.
  bool tts_done_pending_ = false;
  uint64_t tts_done_epoch_ = 0;
  std::string tts_done_request_id_;

  bool tts_playback_done_pending_ = false;
  uint64_t tts_playback_done_epoch_ = 0;
  std::string tts_playback_done_request_id_;
};

}  // namespace k1muse_multimodal_supervisor
