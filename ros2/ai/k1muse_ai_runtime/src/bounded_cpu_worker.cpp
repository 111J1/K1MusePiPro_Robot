#include "k1muse_ai_runtime/bounded_cpu_worker.hpp"

#include <exception>
#include <utility>

namespace k1muse_ai_runtime
{

BoundedCpuWorker::BoundedCpuWorker(
  JobModule module, std::size_t capacity,
  std::chrono::milliseconds stop_timeout)
: module_(module), capacity_(capacity), stop_timeout_(stop_timeout)
{
  stats_.accepting = true;
}

BoundedCpuWorker::~BoundedCpuWorker()
{
  reject_new_jobs();
  cancel_pending();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
    if (active_token_) {
      active_token_->request_stop();
    }
  }
  work_available_.notify_all();
  if (worker_.joinable()) {
    // Job execution is contractually cooperative; never detach from owned state.
    worker_.join();
  }
}

bool BoundedCpuWorker::submit(InferenceJob job)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (job.module != module_ || !job_contract_valid(job) ||
      canonical_provider(job.module) != ProviderClass::Cpu ||
      job.provider_class != ProviderClass::Cpu ||
      job.priority != JobPriority::Cpu)
    {
      ++stats_.rejected_metadata;
      return false;
    }
    if (!stats_.accepting) {
      return false;
    }
    if (queue_.size() >= capacity_) {
      ++stats_.rejected_full;
      return false;
    }
    queue_.push_back(std::move(job));
    ++stats_.submitted;
    stats_.queued = queue_.size();
  }
  work_available_.notify_one();
  notify_state();
  return true;
}

void BoundedCpuWorker::start()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      return;
    }
    stop_requested_ = false;
    worker_stopped_ = false;
    running_ = true;
    stats_.accepting = true;
    worker_ = std::thread(&BoundedCpuWorker::worker_loop, this);
  }
  notify_state();
}

void BoundedCpuWorker::reject_new_jobs()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.accepting = false;
  }
  notify_state();
}

bool BoundedCpuWorker::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.accepting = false;
    stop_requested_ = true;
    stats_.dropped += queue_.size();
    queue_.clear();
    stats_.queued = 0;
    if (active_token_) {
      active_token_->request_stop();
    }
    if (!running_) {
      return true;
    }
  }
  work_available_.notify_all();
  notify_state();
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!stopped_.wait_for(lock, stop_timeout_, [this]() {return worker_stopped_;})) {
      return false;
    }
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    active_token_.reset();
  }
  notify_state();
  return true;
}

std::size_t BoundedCpuWorker::cancel_pending()
{
  std::size_t count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    count = queue_.size();
    queue_.clear();
    stats_.dropped += count;
    stats_.queued = 0;
  }
  notify_state();
  return count;
}

QueueStats BoundedCpuWorker::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = stats_;
  result.queued = queue_.size();
  return result;
}

bool BoundedCpuWorker::has_live_thread() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

JobModule BoundedCpuWorker::module() const noexcept
{
  return module_;
}

void BoundedCpuWorker::set_state_callback(std::function<void()> callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_callback_ = std::move(callback);
}

void BoundedCpuWorker::worker_loop()
{
  while (true) {
    InferenceJob job;
    auto token = std::make_shared<CancellationToken>();
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_available_.wait(lock, [this]() {return stop_requested_ || !queue_.empty();});
      if (stop_requested_) {
        break;
      }
      job = std::move(queue_.front());
      queue_.pop_front();
      active_token_ = token;
      stats_.active_job = job.id;
      stats_.queued = queue_.size();
    }
    notify_state();
    bool executed = false;
    bool committed = false;
    bool dropped = false;
    bool failed = false;
    try {
      if (!job_is_current(job, *token)) {
        dropped = true;
      } else {
        job.execute(*token);
        executed = true;
        committed = job.commit_if_current(*token);
        dropped = !committed;
      }
    } catch (const std::exception &) {
      failed = true;
    } catch (...) {
      failed = true;
    }
    if (token->state() == CancellationState::Committing) {
      token->mark_completed();
    } else if (token->state() == CancellationState::Running) {
      token->request_cancel();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.executed += executed ? 1U : 0U;
      stats_.committed += committed ? 1U : 0U;
      stats_.dropped += dropped ? 1U : 0U;
      stats_.failed += failed ? 1U : 0U;
      stats_.active_job.clear();
      active_token_.reset();
    }
    notify_state();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.active_job.clear();
    active_token_.reset();
    worker_stopped_ = true;
  }
  stopped_.notify_all();
  notify_state();
}

void BoundedCpuWorker::notify_state()
{
  std::function<void()> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback = state_callback_;
  }
  if (callback) {
    callback();
  }
}

}  // namespace k1muse_ai_runtime
