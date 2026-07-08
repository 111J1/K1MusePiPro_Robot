#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "k1muse_ai_runtime/inference_job.hpp"
#include "k1muse_ai_runtime/runtime_scheduler.hpp"

namespace k1muse_ai_runtime
{

class BoundedCpuWorker
{
public:
  BoundedCpuWorker(
    JobModule module, std::size_t capacity,
    std::chrono::milliseconds stop_timeout);
  ~BoundedCpuWorker();

  BoundedCpuWorker(const BoundedCpuWorker &) = delete;
  BoundedCpuWorker & operator=(const BoundedCpuWorker &) = delete;

  bool submit(InferenceJob job);
  void start();
  void reject_new_jobs();
  bool stop();
  std::size_t cancel_pending();
  QueueStats stats() const;
  bool has_live_thread() const;
  JobModule module() const noexcept;
  void set_state_callback(std::function<void()> callback);

private:
  void worker_loop();
  void notify_state();

  const JobModule module_;
  const std::size_t capacity_;
  const std::chrono::milliseconds stop_timeout_;
  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::condition_variable stopped_;
  std::deque<InferenceJob> queue_;
  std::thread worker_;
  std::shared_ptr<CancellationToken> active_token_;
  std::function<void()> state_callback_;
  QueueStats stats_;
  bool running_{false};
  bool stop_requested_{false};
  bool worker_stopped_{true};
};

}  // namespace k1muse_ai_runtime
