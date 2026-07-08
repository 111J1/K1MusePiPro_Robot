#include "k1muse_ai_runtime/runtime_profile_manager.hpp"

#include <algorithm>
#include <stdexcept>

namespace k1muse_ai_runtime
{

// ── RuntimeProfile helpers ────────────────────────────────────

std::string RuntimeProfile::mode_to_string(Mode m)
{
  switch (m) {
    case Mode::NORMAL:     return "NORMAL";
    case Mode::GUARD:      return "GUARD";
    case Mode::PATROL:     return "PATROL";
    case Mode::VOICE_ONLY: return "VOICE_ONLY";
    case Mode::STANDBY:    return "STANDBY";
  }
  return "UNKNOWN";
}

RuntimeProfile::Mode RuntimeProfile::mode_from_string(const std::string& s)
{
  if (s == "NORMAL")     return Mode::NORMAL;
  if (s == "GUARD")      return Mode::GUARD;
  if (s == "PATROL")     return Mode::PATROL;
  if (s == "VOICE_ONLY") return Mode::VOICE_ONLY;
  if (s == "STANDBY")    return Mode::STANDBY;
  throw std::invalid_argument("unknown mode: " + s);
}

// ── RuntimeProfileManager ─────────────────────────────────────

RuntimeProfileManager::RuntimeProfileManager()
{
  current_profile_ = profile_for_mode(RuntimeProfile::Mode::NORMAL);
}

bool RuntimeProfileManager::request_profile(RuntimeProfile::Mode mode)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Reject if already transitioning.
  if (state_ != ApplyState::IDLE && state_ != ApplyState::APPLIED) {
    return false;
  }

  // No-op if already in target mode.
  if (mode == current_mode_ && state_ == ApplyState::APPLIED) {
    return false;
  }

  RuntimeProfile target = profile_for_mode(mode);
  if (!validate_profile(target)) {
    return false;
  }

  previous_mode_ = current_mode_;
  target_mode_ = mode;
  target_profile_ = std::move(target);
  state_ = ApplyState::REQUESTED;
  drain_complete_ = false;
  warmup_complete_ = false;
  last_error_.clear();

  return true;
}

RuntimeProfileManager::ApplyState RuntimeProfileManager::advance()
{
  std::lock_guard<std::mutex> lock(mutex_);

  switch (state_) {
    case ApplyState::REQUESTED:
      state_ = ApplyState::APPLYING;
      break;

    case ApplyState::APPLYING:
      // Config applied — now drain in-flight vision jobs.
      state_ = ApplyState::DRAINING;
      break;

    case ApplyState::DRAINING:
      // Waiting for on_drain_complete() from vision pipeline.
      if (drain_complete_) {
        state_ = ApplyState::WARMING;
      }
      break;

    case ApplyState::WARMING:
      // Waiting for on_warmup_complete() from detector pipeline.
      if (warmup_complete_) {
        state_ = ApplyState::COMMITTING;
      }
      break;

    case ApplyState::COMMITTING:
      // Commit the profile switch.
      current_mode_ = target_mode_;
      current_profile_ = target_profile_;
      state_ = ApplyState::APPLIED;
      break;

    case ApplyState::FAILED:
      // Rollback already handled.
      break;

    case ApplyState::APPLIED:
    case ApplyState::IDLE:
      break;
  }

  return state_;
}

void RuntimeProfileManager::on_drain_complete()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == ApplyState::DRAINING) {
    drain_complete_ = true;
  }
}

void RuntimeProfileManager::on_warmup_complete()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == ApplyState::WARMING) {
    warmup_complete_ = true;
  }
}

void RuntimeProfileManager::on_failure(const std::string& reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_error_ = reason;
  state_ = ApplyState::FAILED;
  rollback();
}

void RuntimeProfileManager::rollback()
{
  // Called under lock.
  target_mode_ = previous_mode_;
  target_profile_ = profile_for_mode(previous_mode_);
  drain_complete_ = false;
  warmup_complete_ = false;

  // Immediate rollback — no async phases needed to return to
  // the previous (already-warm) profile.
  current_mode_ = previous_mode_;
  current_profile_ = target_profile_;
  state_ = ApplyState::APPLIED;
}

// ── Query ─────────────────────────────────────────────────────

RuntimeProfileManager::ApplyState RuntimeProfileManager::apply_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

RuntimeProfile::Mode RuntimeProfileManager::current_mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_mode_;
}

RuntimeProfile::Mode RuntimeProfileManager::target_mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_mode_;
}

const RuntimeProfile& RuntimeProfileManager::current_profile() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_profile_;
}

const RuntimeProfile& RuntimeProfileManager::target_profile() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_profile_;
}

std::string RuntimeProfileManager::state_name(ApplyState s) const
{
  switch (s) {
    case ApplyState::IDLE:       return "IDLE";
    case ApplyState::REQUESTED:  return "REQUESTED";
    case ApplyState::APPLYING:   return "APPLYING";
    case ApplyState::DRAINING:   return "DRAINING";
    case ApplyState::WARMING:    return "WARMING";
    case ApplyState::COMMITTING: return "COMMITTING";
    case ApplyState::APPLIED:    return "APPLIED";
    case ApplyState::FAILED:     return "FAILED";
  }
  return "UNKNOWN";
}

std::string RuntimeProfileManager::last_error() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

// ── Private ───────────────────────────────────────────────────

RuntimeProfile RuntimeProfileManager::profile_for_mode(
    RuntimeProfile::Mode mode) const
{
  RuntimeProfile p;
  p.mode = mode;
  p.name = RuntimeProfile::mode_to_string(mode);

  switch (mode) {
    case RuntimeProfile::Mode::NORMAL:
      p.vision_schedule = {
        {"yolov8n", 500, 0, true, true, "vision"},
      };
      break;

    case RuntimeProfile::Mode::GUARD:
      p.vision_schedule = {
        {"yolov8n-pose", 500, 0, true, true, "vision"},
        {"yolov8_fire",  500, 0, true, true, "vision"},
      };
      break;

    case RuntimeProfile::Mode::PATROL:
      p.vision_schedule = {
        {"yolov8n",       500, 0, true, true, "vision"},
        {"yolov8n-pose",  500, 0, true, true, "vision"},
        {"yolov8_fire",   500, 0, true, true, "vision"},
      };
      break;

    case RuntimeProfile::Mode::VOICE_ONLY:
      p.tts_enabled = true;
      p.wakeword_enabled = true;
      // vision_schedule remains empty.
      break;

    case RuntimeProfile::Mode::STANDBY:
      p.tts_enabled = false;
      p.wakeword_enabled = true;
      break;
  }

  return p;
}

bool RuntimeProfileManager::validate_profile(
    const RuntimeProfile& profile)
{
  // Detect duplicate detector names.
  for (size_t i = 0; i < profile.vision_schedule.size(); ++i) {
    for (size_t j = i + 1; j < profile.vision_schedule.size(); ++j) {
      if (profile.vision_schedule[i].name ==
          profile.vision_schedule[j].name) {
        last_error_ = "duplicate detector: " +
                      profile.vision_schedule[i].name;
        return false;
      }
    }
  }

  // All detectors must have non-empty names.
  for (const auto& dp : profile.vision_schedule) {
    if (dp.name.empty()) {
      last_error_ = "detector with empty name";
      return false;
    }
  }

  return true;
}

}  // namespace k1muse_ai_runtime
