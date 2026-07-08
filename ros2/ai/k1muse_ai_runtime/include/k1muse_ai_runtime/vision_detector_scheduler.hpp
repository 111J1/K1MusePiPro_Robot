#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "k1muse_ai_runtime/runtime_profile_manager.hpp"
#include "k1muse_ai_runtime/voice_exclusive_guard.hpp"

namespace k1muse_ai_runtime
{

/// Fair round-robin scheduler for multiple vision detectors.
///
/// Each detector has a minimum scheduling interval.  The scheduler
/// picks the next detector whose interval has elapsed, cycling
/// through the schedule list in order.
///
/// When VoiceExclusiveGuard reports voice_active(), no detector
/// will be selected — the vision pipeline is expected to drain.
///
/// Thread-safe.
class VisionDetectorScheduler
{
public:
  struct DetectorState
  {
    std::string name;
    std::chrono::steady_clock::time_point last_submitted;
    std::chrono::steady_clock::time_point last_completed;
    int64_t last_latency_ms{0};
    int consecutive_failures{0};
    int total_submitted{0};
    int total_completed{0};
  };

  VisionDetectorScheduler() = default;

  /// Update the schedule from a new RuntimeProfile.
  /// Resets round-robin position and detector state for detectors
  /// not present in the new schedule.
  void update_schedule(const std::vector<DetectorPolicy>& policies);

  /// Pick the next detector to run.
  ///
  /// `guard` is consulted before selection — if voice is active,
  /// returns std::nullopt.
  ///
  /// `now` should be steady_clock::now().
  ///
  /// Returns the name of the detector to run, or std::nullopt if
  /// no detector is due or voice has priority.
  std::optional<std::string> pick_next(
      const VoiceExclusiveGuard& guard,
      std::chrono::steady_clock::time_point now);

  /// Record that a detector submission was made.
  void on_submitted(const std::string& name,
                    std::chrono::steady_clock::time_point when);

  /// Record that a detector job completed.
  void on_completed(const std::string& name,
                    std::chrono::steady_clock::time_point when,
                    int64_t latency_ms, bool success);

  /// Get per-detector state for diagnostics.
  DetectorState state(const std::string& name) const;
  std::vector<DetectorState> all_states() const;

  /// Reset all state.
  void reset();

private:
  mutable std::mutex mutex_;

  std::vector<DetectorPolicy> policies_;
  std::unordered_map<std::string, DetectorState> states_;
  size_t round_robin_index_{0};
};

}  // namespace k1muse_ai_runtime
