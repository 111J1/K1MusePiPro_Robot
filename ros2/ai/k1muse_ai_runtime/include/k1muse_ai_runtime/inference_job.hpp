#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace k1muse_ai_runtime
{

enum class CancellationState : uint8_t
{
  Running,
  Cancelled,
  Committing,
  Completed,
};

class CancellationToken
{
public:
  CancellationToken()
  : state_(std::make_shared<std::atomic<CancellationState>>(CancellationState::Running))
  {
  }

  bool request_cancel() const noexcept
  {
    auto expected = CancellationState::Running;
    return state_->compare_exchange_strong(expected, CancellationState::Cancelled);
  }

  bool request_stop() const noexcept {return request_cancel();}

  bool try_begin_commit() const noexcept
  {
    auto expected = CancellationState::Running;
    return state_->compare_exchange_strong(expected, CancellationState::Committing);
  }

  void mark_completed() const noexcept
  {
    auto expected = CancellationState::Committing;
    state_->compare_exchange_strong(expected, CancellationState::Completed);
  }

  CancellationState state() const noexcept {return state_->load();}
  bool stop_requested() const noexcept {return state() == CancellationState::Cancelled;}

private:
  std::shared_ptr<std::atomic<CancellationState>> state_;
};

enum class JobModule
{
  WAKEWORD_CPU,
  VAD_CPU,
  ASR_CPU,
  VISION_CPU,
  TTS_CPU,
  ASR_EP,
  VISION_EP,
  TTS_EP,
};

enum class ProviderClass
{
  Cpu,
  GuardedEp,
};

enum class JobPriority : int
{
  Cpu = 0,
  Vision = 100,
  TtsEp = 200,
  Asr = 300,
};

inline ProviderClass canonical_provider(JobModule module)
{
  switch (module) {
    case JobModule::ASR_EP:
    case JobModule::VISION_EP:
    case JobModule::TTS_EP:
      return ProviderClass::GuardedEp;
    default:
      return ProviderClass::Cpu;
  }
}

inline JobPriority canonical_priority(JobModule module)
{
  switch (module) {
    case JobModule::ASR_EP:
      return JobPriority::Asr;
    case JobModule::TTS_EP:
      return JobPriority::TtsEp;
    case JobModule::VISION_EP:
      return JobPriority::Vision;
    default:
      return JobPriority::Cpu;
  }
}

class CommitGenerationGate
{
public:
  bool update(
    uint64_t epoch, bool wakeword_enabled, bool vad_asr_enabled,
    bool vision_enabled, bool tts_enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch < epoch_) {
      return false;
    }
    epoch_ = epoch;
    wakeword_enabled_ = wakeword_enabled;
    vad_asr_enabled_ = vad_asr_enabled;
    vision_enabled_ = vision_enabled;
    tts_enabled_ = tts_enabled;
    return true;
  }

  bool epoch_is_current(uint64_t epoch) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return epoch_ == epoch;
  }

  bool module_enabled(JobModule module) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return module_enabled_locked(module);
  }

  bool commit_if_current(
    uint64_t epoch, JobModule module,
    std::chrono::steady_clock::time_point deadline,
    const CancellationToken & token,
    const std::function<void()> & commit)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::chrono::steady_clock::now() > deadline ||
      epoch_ != epoch || !module_enabled_locked(module) ||
      !token.try_begin_commit())
    {
      return false;
    }
    try {
      commit();
      token.mark_completed();
      return true;
    } catch (...) {
      token.mark_completed();
      throw;
    }
  }

private:
  bool module_enabled_locked(JobModule module) const
  {
    switch (module) {
      case JobModule::WAKEWORD_CPU:
        return wakeword_enabled_;
      case JobModule::VAD_CPU:
      case JobModule::ASR_CPU:
      case JobModule::ASR_EP:
        return vad_asr_enabled_;
      case JobModule::VISION_CPU:
      case JobModule::VISION_EP:
        return vision_enabled_;
      case JobModule::TTS_CPU:
      case JobModule::TTS_EP:
        return tts_enabled_;
    }
    return false;
  }

  mutable std::mutex mutex_;
  uint64_t epoch_{0};
  bool wakeword_enabled_{false};
  bool vad_asr_enabled_{false};
  bool vision_enabled_{false};
  bool tts_enabled_{false};
};

struct InferenceJob
{
  std::string id;
  JobModule module{JobModule::VISION_EP};
  ProviderClass provider_class{ProviderClass::GuardedEp};
  JobPriority priority{JobPriority::Vision};
  uint64_t epoch{0};
  std::chrono::steady_clock::time_point deadline{};
  std::function<bool(uint64_t)> epoch_is_current;
  std::function<bool(JobModule)> module_enabled;
  std::function<bool()> cancel_predicate;
  // execute must poll the token and return by deadline. Workers catch exceptions.
  std::function<void(const CancellationToken &)> execute;
  // Atomically validates current generation/state and performs a non-blocking commit.
  // Workers catch exceptions and never call a separate check followed by a bare commit.
  std::function<bool(const CancellationToken &)> commit_if_current;
  uint64_t sequence{0};
};

inline bool job_contract_valid(const InferenceJob & job)
{
  return !job.id.empty() && job.deadline != std::chrono::steady_clock::time_point{} &&
         static_cast<bool>(job.epoch_is_current) &&
         static_cast<bool>(job.module_enabled) &&
         static_cast<bool>(job.cancel_predicate) &&
         static_cast<bool>(job.execute) &&
         static_cast<bool>(job.commit_if_current);
}

inline bool job_is_current(const InferenceJob & job, const CancellationToken & token)
{
  if (!job_contract_valid(job) || token.stop_requested() ||
    std::chrono::steady_clock::now() > job.deadline)
  {
    return false;
  }
  return job.epoch_is_current(job.epoch) &&
         job.module_enabled(job.module) &&
         !job.cancel_predicate();
}

}  // namespace k1muse_ai_runtime
