#include "k1muse_ai_runtime/runtime_core.hpp"

namespace k1muse_ai_runtime
{

RuntimeCore::RuntimeCore()
{
  // Apply the default NORMAL profile to the vision pipeline.
  vision_.apply_profile(profile_mgr_.current_profile());
}

// ── Convenience ───────────────────────────────────────────────

bool RuntimeCore::request_profile(RuntimeProfile::Mode mode)
{
  bool ok = profile_mgr_.request_profile(mode);
  if (ok) {
    // Apply the target profile's detector schedule immediately
    // so the scheduler starts using it.  The full apply/ack flow
    // handles drain/warmup separately.
    vision_.apply_profile(profile_mgr_.target_profile());
  }
  return ok;
}

RuntimeProfileManager::ApplyState RuntimeCore::advance_profile()
{
  return profile_mgr_.advance();
}

void RuntimeCore::on_drain_complete()
{
  profile_mgr_.on_drain_complete();
}

void RuntimeCore::on_warmup_complete()
{
  profile_mgr_.on_warmup_complete();
}

void RuntimeCore::on_profile_failure(const std::string& reason)
{
  profile_mgr_.on_failure(reason);
}

// ── Control → Voice guard mapping ─────────────────────────────

void RuntimeCore::apply_control(const ControlSnapshot& ctrl)
{
  // Map Supervisor interaction states to voice guard actions.
  // States where voice requires EP:
  //   - LISTENING (VAD + ASR active)
  //   - TTS_RUNNING (TTS synthesis)
  //
  // States where voice releases EP:
  //   - IDLE (wakeword + vision active)
  //   - WAKE_ACK (playback only)
  //   - INTENT_PROCESSING (CPU only)
  //   - TARGETING (vision active, voice idle)

  const std::string& state = ctrl.interaction_state_name;

  if (state == "LISTENING" || state == "TTS_RUNNING") {
    voice_guard_.request_voice();
  } else if (state == "IDLE" || state == "WAKE_ACK" ||
             state == "INTENT_PROCESSING" || state == "TARGETING") {
    voice_guard_.release_voice();
  }
  // EMERGENCY_OR_FAULT, BOOT: leave voice guard as-is.
}

}  // namespace k1muse_ai_runtime
