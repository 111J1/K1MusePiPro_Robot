#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

namespace k1muse_ai_runtime
{

/// Alert confirmation and cooldown logic.  Pure C++ — no ROS dependency.
///
/// Alerts require N consecutive positive frames before firing
/// (confirmation), and a cooldown period before firing again for the
/// same alert type.
///
/// Thread-safe.
class AlertEventPublisher
{
public:
  enum class AlertType : uint8_t
  {
    kFall = 0,
    kFire = 1,
    kSmoke = 2,
    kAnomaly = 3,
  };

  struct Config
  {
    int fall_confirm_frames = 5;        // consecutive frames to confirm fall
    int fire_confirm_frames = 3;        // consecutive frames to confirm fire
    int anomaly_confirm_frames = 10;    // consecutive frames to confirm anomaly
    int fall_cooldown_ms = 30'000;      // 30 s between fall alerts
    int fire_cooldown_ms = 30'000;      // 30 s between fire alerts
    int smoke_cooldown_ms = 3'000;      // 3 s between smoke alerts
    int anomaly_cooldown_ms = 120'000;  // 120 s between anomaly alerts
  };

  struct AlertInfo
  {
    AlertType type;
    std::string alert_id;
    std::string detector_source;
    int confirmed_frames{0};
    float window_confidence{0.0f};
    std::chrono::steady_clock::time_point first_seen;
    std::chrono::steady_clock::time_point last_seen;
    bool fired{false};  // true when this alert should be published
  };

  AlertEventPublisher() = default;
  explicit AlertEventPublisher(Config config);

  /// Feed a detection result from a detector pipeline.
  /// `positive` = true means the detector saw the alert condition.
  /// `confidence` is the detector's confidence [0.0, 1.0].
  /// `detector_source` identifies which detector (e.g. "yolov8_fire").
  ///
  /// Returns the AlertInfo if the alert should be published,
  /// otherwise returns an info with fired=false.
  AlertInfo feed(AlertType type, bool positive, float confidence,
                 const std::string& detector_source);

  /// Force-reset any in-progress confirmation window for `type`.
  void reset(AlertType type);

  /// Check whether `type` is currently in cooldown.
  bool in_cooldown(AlertType type) const;

private:
  struct AlertState
  {
    int consecutive_positives{0};
    int consecutive_negatives{0};
    float confidence_sum{0.0f};
    std::chrono::steady_clock::time_point window_start;
    std::chrono::steady_clock::time_point last_positive;
    std::chrono::steady_clock::time_point last_fired;
    std::string detector_source;
    bool in_cooldown{false};
    int alert_counter{0};  // for generating unique alert_ids
  };

  int confirm_frames(AlertType type) const;
  int cooldown_ms(AlertType type) const;
  std::string alert_type_name(AlertType type) const;

  Config config_;
  mutable std::mutex mutex_;
  std::unordered_map<AlertType, AlertState> states_;
};

}  // namespace k1muse_ai_runtime
