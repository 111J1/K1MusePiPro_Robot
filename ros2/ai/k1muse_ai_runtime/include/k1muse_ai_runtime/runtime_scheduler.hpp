#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "k1muse_ai_runtime/inference_job.hpp"
#include "k1muse_ai_runtime/resource_guard.hpp"

namespace k1muse_ai_runtime
{

struct QueueStats
{
  uint64_t submitted{0};
  uint64_t executed{0};
  uint64_t committed{0};
  uint64_t dropped{0};
  uint64_t failed{0};
  uint64_t rejected_full{0};
  uint64_t rejected_metadata{0};
  std::size_t queued{0};
  bool accepting{false};
  std::string active_job;
};

class RuntimeScheduler
{
public:
  RuntimeScheduler(
    ResourceGuard & guard, std::size_t capacity,
    std::chrono::milliseconds stop_timeout);
  ~RuntimeScheduler();

  RuntimeScheduler(const RuntimeScheduler &) = delete;
  RuntimeScheduler & operator=(const RuntimeScheduler &) = delete;

  bool submit(InferenceJob job);
  void start();
  void reject_new_jobs();
  bool stop();
  std::size_t cancel_pending();
  QueueStats stats() const;
  bool has_live_thread() const;
  void set_state_callback(std::function<void()> callback);

private:
  struct Compare
  {
    bool operator()(const InferenceJob & lhs, const InferenceJob & rhs) const;
  };

  void worker_loop();
  void notify_state();

  ResourceGuard & guard_;
  const std::size_t capacity_;
  const std::chrono::milliseconds stop_timeout_;
  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::condition_variable stopped_;
  std::priority_queue<InferenceJob, std::vector<InferenceJob>, Compare> queue_;
  std::thread worker_;
  std::shared_ptr<CancellationToken> active_token_;
  std::function<void()> state_callback_;
  QueueStats stats_;
  uint64_t next_sequence_{0};
  bool running_{false};
  bool stop_requested_{false};
  bool worker_stopped_{true};
};

}  // namespace k1muse_ai_runtime
