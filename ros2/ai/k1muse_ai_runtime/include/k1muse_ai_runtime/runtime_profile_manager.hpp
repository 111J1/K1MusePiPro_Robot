#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

/// Per-detector scheduling policy.
struct DetectorPolicy
{
  std::string name;              // detector name (e.g. "yolov8n", "yolov8n-pose", "yolov8_fire")
  int interval_ms = 500;         // minimum interval between submissions
  int priority = 0;              // higher = scheduled first
  bool requires_ep = true;       // true = must go through ResourceGuard
  bool enabled = true;           // may be disabled per-profile
  std::string concurrency_group; // "vision" or "voice"
};

/// Runtime profile — AI model resource configuration only.
/// Does NOT determine robot behaviour (that's Supervisor's job).
struct RuntimeProfile
{
  enum class Mode : uint8_t
  {
    NORMAL = 0,      // YOLO object detection only
    GUARD = 1,       // Pose + Fire serial switching, alerts enabled
    PATROL = 2,      // All detectors, SLAM integration planned
    VOICE_ONLY = 3,  // All vision disabled, voice only
    STANDBY = 4,     // Minimal — wakeword only
  };

  Mode mode = Mode::NORMAL;
  std::string name;

  bool vision_parallel_enabled = false;       // default: serial only
  std::string vision_ep_profile = "vision_serial_2t";
  std::string voice_ep_profile = "voice_serial_2t";
  std::string fallback_vision_ep_profile;     // used when EP profile switch fails

  bool tts_enabled = true;
  bool wakeword_enabled = true;

  std::vector<DetectorPolicy> vision_schedule; // detectors to run in this mode

  static std::string mode_to_string(Mode m);
  static Mode mode_from_string(const std::string& s);
};

/// Async two-phase profile application state machine.
///
/// REQUESTED → APPLYING → DRAINING → WARMING → COMMITTING → APPLIED
///                                              ↓
///                                (any phase can fail)
///                                              ↓
///                                ROLLBACK → previous profile
///
/// The Supervisor publishes a new profile via /ai_runtime/control.
/// This manager orchestrates the transition, calling into pipelines
/// to drain/warmup, and reporting progress via /ai_runtime/state.
///
/// Thread-safe.
class RuntimeProfileManager
{
public:
  enum class ApplyState : uint8_t
  {
    IDLE,          // no transition in progress
    REQUESTED,     // new profile requested, not yet started
    APPLYING,      // applying config changes
    DRAINING,      // draining in-flight vision jobs
    WARMING,       // warming up new detectors
    COMMITTING,    // final checks, about to switch
    APPLIED,       // transition complete
    FAILED,        // transition failed, rolling back
  };

  RuntimeProfileManager();

  /// Request a profile switch.  If a transition is already in
  /// progress, returns false (caller should retry later).
  bool request_profile(RuntimeProfile::Mode mode);

  /// Advance state machine.  Caller (e.g. a timer or callback)
  /// drives the transition forward after each async phase completes.
  /// Returns the new state.
  ApplyState advance();

  /// Signal that the drain phase is complete (all in-flight vision
  /// jobs have finished and released their EP leases).
  void on_drain_complete();

  /// Signal that the warmup phase is complete (new detectors are
  /// loaded and ready).
  void on_warmup_complete();

  /// Signal a failure in the current transition.  Triggers rollback
  /// to the previous profile.
  void on_failure(const std::string& reason);

  /// Abandon the transition and return to the previous profile.
  void rollback();

  // ── Query ──

  ApplyState apply_state() const;
  RuntimeProfile::Mode current_mode() const;
  RuntimeProfile::Mode target_mode() const;
  const RuntimeProfile& current_profile() const;
  const RuntimeProfile& target_profile() const;
  std::string state_name(ApplyState s) const;
  std::string last_error() const;

private:
  RuntimeProfile profile_for_mode(RuntimeProfile::Mode mode) const;
  bool validate_profile(const RuntimeProfile& profile);

  mutable std::mutex mutex_;

  RuntimeProfile::Mode current_mode_{RuntimeProfile::Mode::NORMAL};
  RuntimeProfile::Mode target_mode_{RuntimeProfile::Mode::NORMAL};
  RuntimeProfile::Mode previous_mode_{RuntimeProfile::Mode::NORMAL};

  RuntimeProfile current_profile_;
  RuntimeProfile target_profile_;

  ApplyState state_{ApplyState::IDLE};
  bool drain_complete_{false};
  bool warmup_complete_{false};
  std::string last_error_;
};

}  // namespace k1muse_ai_runtime
