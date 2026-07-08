#include "k1muse_ai_runtime/vision_detector_scheduler.hpp"

#include <algorithm>

namespace k1muse_ai_runtime
{

void VisionDetectorScheduler::update_schedule(
    const std::vector<DetectorPolicy>& policies)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Build a set of names in the new schedule.
  std::unordered_map<std::string, bool> new_names;
  for (const auto& p : policies) {
    new_names[p.name] = true;
  }

  // Remove state for detectors not in the new schedule.
  auto it = states_.begin();
  while (it != states_.end()) {
    if (new_names.find(it->first) == new_names.end()) {
      it = states_.erase(it);
    } else {
      ++it;
    }
  }

  // Ensure all new policies have state entries.
  for (const auto& p : policies) {
    if (states_.find(p.name) == states_.end()) {
      DetectorState ds;
      ds.name = p.name;
      states_[p.name] = ds;
    }
  }

  policies_ = policies;
  // Reset round-robin to start of list.
  round_robin_index_ = 0;
}

std::optional<std::string> VisionDetectorScheduler::pick_next(
    const VoiceExclusiveGuard& guard,
    std::chrono::steady_clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Voice has priority — don't submit new vision.
  if (guard.voice_active()) {
    return std::nullopt;
  }

  if (policies_.empty()) {
    return std::nullopt;
  }

  // Try up to one full cycle to find a detector whose interval has
  // elapsed and which is enabled.
  for (size_t attempt = 0; attempt < policies_.size(); ++attempt) {
    size_t idx = round_robin_index_ % policies_.size();
    round_robin_index_ = (round_robin_index_ + 1) % policies_.size();

    const auto& policy = policies_[idx];
    if (!policy.enabled) continue;
    if (!policy.requires_ep) continue;  // CPU detectors not scheduled here

    const auto& state = states_[policy.name];
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - state.last_submitted);

    if (elapsed.count() >= policy.interval_ms) {
      return policy.name;
    }
  }

  // No detector is due yet.
  return std::nullopt;
}

void VisionDetectorScheduler::on_submitted(
    const std::string& name, std::chrono::steady_clock::time_point when)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(name);
  if (it == states_.end()) return;
  it->second.last_submitted = when;
  ++it->second.total_submitted;
}

void VisionDetectorScheduler::on_completed(
    const std::string& name, std::chrono::steady_clock::time_point when,
    int64_t latency_ms, bool success)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(name);
  if (it == states_.end()) return;
  it->second.last_completed = when;
  it->second.last_latency_ms = latency_ms;
  ++it->second.total_completed;
  if (success) {
    it->second.consecutive_failures = 0;
  } else {
    ++it->second.consecutive_failures;
  }
}

VisionDetectorScheduler::DetectorState VisionDetectorScheduler::state(
    const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(name);
  if (it != states_.end()) return it->second;
  DetectorState ds;
  ds.name = name;
  return ds;
}

std::vector<VisionDetectorScheduler::DetectorState>
VisionDetectorScheduler::all_states() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DetectorState> result;
  result.reserve(states_.size());
  for (const auto& [name, ds] : states_) {
    result.push_back(ds);
  }
  return result;
}

void VisionDetectorScheduler::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  states_.clear();
  policies_.clear();
  round_robin_index_ = 0;
}

}  // namespace k1muse_ai_runtime
