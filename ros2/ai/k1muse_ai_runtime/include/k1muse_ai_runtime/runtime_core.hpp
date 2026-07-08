#pragma once

#include <memory>
#include <string>

#include "k1muse_ai_runtime/alert_publisher.hpp"
#include "k1muse_ai_runtime/control_snapshot.hpp"
#include "k1muse_ai_runtime/pipelines/vision_pipeline.hpp"
#include "k1muse_ai_runtime/runtime_profile_manager.hpp"
#include "k1muse_ai_runtime/voice_exclusive_guard.hpp"

namespace k1muse_ai_runtime
{

/// Ownership container for V2 runtime components.
///
/// Holds the VisionPipeline (multi-detector), VoiceExclusiveGuard,
/// RuntimeProfileManager, and exposes the shared guard so voice
/// and vision paths can coordinate.
///
/// All ROS-specific wiring (publishers, subscriptions) remains in
/// AiRuntimeNode.  This class is pure logic.
class RuntimeCore
{
public:
  RuntimeCore();
  ~RuntimeCore() = default;

  RuntimeCore(const RuntimeCore&) = delete;
  RuntimeCore& operator=(const RuntimeCore&) = delete;

  // ── Components ──

  VisionPipeline& vision() { return vision_; }
  const VisionPipeline& vision() const { return vision_; }

  RuntimeProfileManager& profile_mgr() { return profile_mgr_; }
  const RuntimeProfileManager& profile_mgr() const { return profile_mgr_; }

  /// Shared guard — voice pipeline calls request_voice/release_voice;
  /// vision pipeline reads can_submit_vision/voice_active.
  VoiceExclusiveGuard& voice_guard() { return voice_guard_; }
  const VoiceExclusiveGuard& voice_guard() const { return voice_guard_; }

  // ── Convenience ──

  /// Request a runtime profile switch.
  bool request_profile(RuntimeProfile::Mode mode);

  /// Advance the profile state machine.  Returns the new state.
  RuntimeProfileManager::ApplyState advance_profile();

  /// Signal that the current vision jobs have drained.
  void on_drain_complete();

  /// Signal that new detectors have warmed up.
  void on_warmup_complete();

  /// Signal a profile transition failure.
  void on_profile_failure(const std::string& reason);

  /// Apply a ControlSnapshot from the Supervisor.  Maps interaction
  /// state flags to voice guard actions.
  void apply_control(const ControlSnapshot& ctrl);

private:
  VisionPipeline vision_;
  RuntimeProfileManager profile_mgr_;
  VoiceExclusiveGuard voice_guard_;  // shared between voice + vision
};

}  // namespace k1muse_ai_runtime
