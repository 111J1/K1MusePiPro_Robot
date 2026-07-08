#include "k1muse_multimodal_supervisor/interaction_state_machine.hpp"

namespace k1muse_multimodal_supervisor {

// ---------------------------------------------------------------------------
// state_name
// ---------------------------------------------------------------------------

const char* state_name(State s) {
  switch (s) {
    case State::BOOT:                return "BOOT";
    case State::IDLE:                return "IDLE";
    case State::WAKE_ACK:            return "WAKE_ACK";
    case State::LISTENING:           return "LISTENING";
    case State::INTENT_PROCESSING:   return "INTENT_PROCESSING";
    case State::TTS_RUNNING:         return "TTS_RUNNING";
    case State::TARGETING:           return "TARGETING";
    case State::EMERGENCY_OR_FAULT:  return "EMERGENCY_OR_FAULT";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// InteractionStateMachine
// ---------------------------------------------------------------------------

InteractionStateMachine::InteractionStateMachine() : state_(State::BOOT) {}

State InteractionStateMachine::current_state() const { return state_; }

void InteractionStateMachine::set_state(State state) { state_ = state; }

bool InteractionStateMachine::update_readiness(bool runtime_ready,
                                                bool audio_ready,
                                                bool intent_ready) {
  runtime_ready_ = runtime_ready;
  audio_ready_   = audio_ready;
  intent_ready_  = intent_ready;
  return (state_ == State::BOOT && runtime_ready_ && audio_ready_ && intent_ready_);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

bool epoch_matches(uint64_t active, uint64_t event) {
  return active == event && event != 0;
}

bool request_id_matches(const std::string& active, const std::string& event) {
  return !active.empty() && active == event;
}

bool trace_id_matches(const std::string& active, const std::string& event) {
  return !active.empty() && active == event;
}

TransitionResult reject(State current, const char* reason) {
  return TransitionResult{current, false, reason};
}

TransitionResult accept(State next, const char* reason) {
  return TransitionResult{next, true, reason};
}

}  // namespace

// ---------------------------------------------------------------------------
// process
// ---------------------------------------------------------------------------

TransitionResult InteractionStateMachine::process(Event event,
                                                   const TransitionContext& ctx) {
  switch (event) {
    // ------------------------------------------------------------------
    // Readiness events
    // ------------------------------------------------------------------
    case Event::RUNTIME_READY_CHANGED:
    case Event::AUDIO_READY_CHANGED:
    case Event::INTENT_READY_CHANGED: {
      runtime_ready_ = ctx.runtime_ready;
      audio_ready_   = ctx.audio_ready;
      intent_ready_  = ctx.intent_ready;
      if (state_ == State::BOOT && runtime_ready_ && audio_ready_ && intent_ready_) {
        return accept(State::IDLE, "all subsystems ready");
      }
      return reject(state_, "not all subsystems ready yet");
    }

    // ------------------------------------------------------------------
    // IDLE -> WAKE_ACK
    // ------------------------------------------------------------------
    case Event::WAKEWORD_ACCEPTED: {
      if (state_ != State::IDLE) {
        return reject(state_, "WAKEWORD_ACCEPTED only valid in IDLE");
      }
      if (!epoch_matches(ctx.active_epoch, ctx.event_epoch)) {
        return reject(state_, "stale epoch");
      }
      return accept(State::WAKE_ACK, "wakeword accepted");
    }

    // ------------------------------------------------------------------
    // IDLE -> TARGETING
    // ------------------------------------------------------------------
    case Event::TARGET_REQUEST_RECEIVED: {
      if (state_ != State::IDLE) {
        return reject(state_, "TARGET_REQUEST_RECEIVED only valid in IDLE");
      }
      return accept(State::TARGETING, "target request received");
    }

    // ------------------------------------------------------------------
    // WAKE_ACK -> LISTENING
    // ------------------------------------------------------------------
    case Event::WAKE_ACK_PLAYBACK_DONE: {
      if (state_ != State::WAKE_ACK) {
        return reject(state_, "WAKE_ACK_PLAYBACK_DONE only valid in WAKE_ACK");
      }
      if (!request_id_matches(ctx.active_request_id, ctx.event_request_id) ||
          !epoch_matches(ctx.active_epoch, ctx.event_epoch)) {
        return reject(state_, "stale request/epoch");
      }
      return accept(State::LISTENING, "wake-ack playback done");
    }

    case Event::WAKE_ACK_PLAYBACK_FAILED: {
      if (state_ != State::WAKE_ACK) {
        return reject(state_, "WAKE_ACK_PLAYBACK_FAILED only valid in WAKE_ACK");
      }
      // Fallback policy: proceed to LISTENING even if ack audio failed.
      return accept(State::LISTENING, "wake-ack playback failed, proceeding (fallback)");
    }

    // ------------------------------------------------------------------
    // LISTENING -> INTENT_PROCESSING
    // ------------------------------------------------------------------
    case Event::LISTEN_RESULT_RECEIVED: {
      if (state_ != State::LISTENING) {
        return reject(state_, "LISTEN_RESULT_RECEIVED only valid in LISTENING");
      }
      if (!trace_id_matches(ctx.active_trace_id, ctx.event_trace_id) ||
          !epoch_matches(ctx.active_epoch, ctx.event_epoch)) {
        return reject(state_, "stale trace_id/epoch");
      }
      return accept(State::INTENT_PROCESSING, "listen result received");
    }

    // ------------------------------------------------------------------
    // LISTENING -> IDLE  (no-speech / failure / cancel)
    // ------------------------------------------------------------------
    case Event::NO_SPEECH_TIMEOUT:
    case Event::LISTEN_FAILED:
    case Event::LISTEN_CANCELLED: {
      if (state_ != State::LISTENING) {
        return reject(state_, "listen error event only valid in LISTENING");
      }
      return accept(State::IDLE, "listen session ended without result");
    }

    // ------------------------------------------------------------------
    // INTENT_PROCESSING -> TTS_RUNNING / IDLE
    // ------------------------------------------------------------------
    case Event::INTENT_FINISHED: {
      if (state_ != State::INTENT_PROCESSING) {
        return reject(state_, "INTENT_FINISHED only valid in INTENT_PROCESSING");
      }
      if (!request_id_matches(ctx.active_request_id, ctx.event_request_id) ||
          !epoch_matches(ctx.active_epoch, ctx.event_epoch)) {
        return reject(state_, "stale request_id/epoch");
      }
      if (!ctx.has_pending_tts) {
        return accept(State::IDLE, "intent finished, no TTS needed");
      }
      return accept(State::TTS_RUNNING, "intent finished");
    }

    case Event::INTENT_FAILED: {
      if (state_ != State::INTENT_PROCESSING) {
        return reject(state_, "INTENT_FAILED only valid in INTENT_PROCESSING");
      }
      if (ctx.has_fallback_tts) {
        return accept(State::TTS_RUNNING, "intent failed, fallback TTS");
      }
      return accept(State::IDLE, "intent failed, no fallback");
    }

    // ------------------------------------------------------------------
    // TTS_RUNNING -> IDLE  (requires both TTS_DONE and TTS_PLAYBACK_DONE)
    // ------------------------------------------------------------------
    case Event::TTS_DONE: {
      if (state_ != State::TTS_RUNNING) {
        return reject(state_, "TTS_DONE only valid in TTS_RUNNING");
      }
      // Check whether playback-done already arrived with matching IDs.
      if (tts_playback_done_pending_ &&
          request_id_matches(tts_playback_done_request_id_, ctx.event_request_id) &&
          epoch_matches(tts_playback_done_epoch_, ctx.event_epoch)) {
        // Both received -- transition.
        tts_done_pending_ = false;
        tts_playback_done_pending_ = false;
        return accept(State::IDLE, "TTS done + playback done");
      }
      // If playback-done is pending but with mismatched IDs, the old one is
      // stale (from a previous lifecycle). Replace it.
      if (tts_playback_done_pending_ &&
          (!request_id_matches(tts_playback_done_request_id_, ctx.event_request_id) ||
           !epoch_matches(tts_playback_done_epoch_, ctx.event_epoch))) {
        // Stale playback-done; overwrite with current TTS-done and wait.
        tts_done_pending_ = true;
        tts_done_epoch_ = ctx.event_epoch;
        tts_done_request_id_ = ctx.event_request_id;
        // Clear stale playback-done.
        tts_playback_done_pending_ = false;
        return reject(state_, "TTS_DONE received, stale playback-done discarded");
      }
      // Store and wait for the other event.
      tts_done_pending_ = true;
      tts_done_epoch_ = ctx.event_epoch;
      tts_done_request_id_ = ctx.event_request_id;
      return reject(state_, "TTS_DONE received, waiting for TTS_PLAYBACK_DONE");
    }

    case Event::TTS_PLAYBACK_DONE: {
      if (state_ != State::TTS_RUNNING) {
        return reject(state_, "TTS_PLAYBACK_DONE only valid in TTS_RUNNING");
      }
      // Check whether TTS_DONE already arrived with matching IDs.
      if (tts_done_pending_ &&
          request_id_matches(tts_done_request_id_, ctx.event_request_id) &&
          epoch_matches(tts_done_epoch_, ctx.event_epoch)) {
        // Both received -- transition.
        tts_done_pending_ = false;
        tts_playback_done_pending_ = false;
        return accept(State::IDLE, "TTS done + playback done");
      }
      // If TTS-done is pending but with mismatched IDs, the old one is
      // stale (from a previous lifecycle). Replace it.
      if (tts_done_pending_ &&
          (!request_id_matches(tts_done_request_id_, ctx.event_request_id) ||
           !epoch_matches(tts_done_epoch_, ctx.event_epoch))) {
        // Stale TTS-done; overwrite with current playback-done and wait.
        tts_playback_done_pending_ = true;
        tts_playback_done_epoch_ = ctx.event_epoch;
        tts_playback_done_request_id_ = ctx.event_request_id;
        // Clear stale TTS-done.
        tts_done_pending_ = false;
        return reject(state_, "TTS_PLAYBACK_DONE received, stale TTS-done discarded");
      }
      // Store and wait for the other event.
      tts_playback_done_pending_ = true;
      tts_playback_done_epoch_ = ctx.event_epoch;
      tts_playback_done_request_id_ = ctx.event_request_id;
      return reject(state_, "TTS_PLAYBACK_DONE received, waiting for TTS_DONE");
    }

    case Event::TTS_FAILED:
    case Event::TTS_PLAYBACK_FAILED: {
      if (state_ != State::TTS_RUNNING) {
        return reject(state_, "TTS failure event only valid in TTS_RUNNING");
      }
      tts_done_pending_ = false;
      tts_playback_done_pending_ = false;
      return accept(State::IDLE, "TTS/playback failed");
    }

    // ------------------------------------------------------------------
    // TARGETING -> IDLE
    // ------------------------------------------------------------------
    case Event::TARGET_RESPONSE_SENT: {
      if (state_ != State::TARGETING) {
        return reject(state_, "TARGET_RESPONSE_SENT only valid in TARGETING");
      }
      return accept(State::IDLE, "target response sent");
    }

    // ------------------------------------------------------------------
    // SYSTEM_FAULT  (any state except EMERGENCY_OR_FAULT)
    // ------------------------------------------------------------------
    case Event::SYSTEM_FAULT: {
      if (state_ == State::EMERGENCY_OR_FAULT) {
        return reject(state_, "already in EMERGENCY_OR_FAULT");
      }
      tts_done_pending_ = false;
      tts_playback_done_pending_ = false;
      return accept(State::EMERGENCY_OR_FAULT, "system fault");
    }

    // ------------------------------------------------------------------
    // FAULT_CLEARED -> IDLE
    // ------------------------------------------------------------------
    case Event::FAULT_CLEARED: {
      if (state_ != State::EMERGENCY_OR_FAULT) {
        return reject(state_, "FAULT_CLEARED only valid in EMERGENCY_OR_FAULT");
      }
      if (!ctx.runtime_ready) {
        return reject(state_, "runtime not ready");
      }
      return accept(State::IDLE, "fault cleared");
    }
  }

  return reject(state_, "unknown event");
}

}  // namespace k1muse_multimodal_supervisor
