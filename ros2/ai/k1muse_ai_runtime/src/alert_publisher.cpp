#include "k1muse_ai_runtime/alert_publisher.hpp"

#include <sstream>

namespace k1muse_ai_runtime
{

AlertEventPublisher::AlertEventPublisher(Config config)
    : config_(std::move(config)) {}
// Default constructor uses Config{} defaults.

AlertEventPublisher::AlertInfo AlertEventPublisher::feed(
    AlertType type, bool positive, float confidence,
    const std::string& detector_source)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto now = std::chrono::steady_clock::now();
  auto& state = states_[type];

  AlertInfo info;
  info.type = type;
  info.detector_source = detector_source;

  // ── Cooldown check ──
  if (state.in_cooldown) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - state.last_fired);
    if (elapsed.count() < cooldown_ms(type)) {
      info.fired = false;
      return info;
    }
    state.in_cooldown = false;
  }

  // ── Update confirmation window ──
  if (state.window_start == std::chrono::steady_clock::time_point{}) {
    state.window_start = now;
  }

  if (positive) {
    ++state.consecutive_positives;
    state.consecutive_negatives = 0;
    state.confidence_sum += confidence;
    state.last_positive = now;
    state.detector_source = detector_source;
  } else {
    ++state.consecutive_negatives;
    // Reset after 3 consecutive negatives (short dropout tolerance).
    if (state.consecutive_negatives >= 3) {
      state.consecutive_positives = 0;
      state.confidence_sum = 0.0f;
      state.window_start = {};
    }
  }

  // ── Confirmation check ──
  int required = confirm_frames(type);
  if (state.consecutive_positives >= required) {
    // Fire alert.
    ++state.alert_counter;

    std::ostringstream id_stream;
    id_stream << alert_type_name(type) << "_" << state.alert_counter;

    info.fired = true;
    info.alert_id = id_stream.str();
    info.confirmed_frames = state.consecutive_positives;
    info.window_confidence =
        state.confidence_sum / static_cast<float>(state.consecutive_positives);
    info.first_seen = state.window_start;
    info.last_seen = state.last_positive;

    // Start cooldown.
    state.in_cooldown = true;
    state.last_fired = now;
    state.consecutive_positives = 0;
    state.confidence_sum = 0.0f;
    state.window_start = {};
  }

  return info;
}

void AlertEventPublisher::reset(AlertType type)
{
  std::lock_guard<std::mutex> lock(mutex_);
  states_.erase(type);
}

bool AlertEventPublisher::in_cooldown(AlertType type) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(type);
  if (it == states_.end()) return false;
  if (!it->second.in_cooldown) return false;
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - it->second.last_fired);
  return elapsed.count() < cooldown_ms(type);
}

int AlertEventPublisher::confirm_frames(AlertType type) const
{
  switch (type) {
    case AlertType::kFall:
      return config_.fall_confirm_frames;
    case AlertType::kFire:
      return config_.fire_confirm_frames;
    case AlertType::kSmoke:
      // Smoke uses same confirm as fire
      return config_.fire_confirm_frames;
    case AlertType::kAnomaly:
      return config_.anomaly_confirm_frames;
  }
  return 3;
}

int AlertEventPublisher::cooldown_ms(AlertType type) const
{
  switch (type) {
    case AlertType::kFall:
      return config_.fall_cooldown_ms;
    case AlertType::kFire:
      return config_.fire_cooldown_ms;
    case AlertType::kSmoke:
      return config_.smoke_cooldown_ms;
    case AlertType::kAnomaly:
      return config_.anomaly_cooldown_ms;
  }
  return 30'000;
}

std::string AlertEventPublisher::alert_type_name(AlertType type) const
{
  switch (type) {
    case AlertType::kFall:    return "fall";
    case AlertType::kFire:    return "fire";
    case AlertType::kSmoke:   return "smoke";
    case AlertType::kAnomaly: return "anomaly";
  }
  return "unknown";
}

}  // namespace k1muse_ai_runtime
