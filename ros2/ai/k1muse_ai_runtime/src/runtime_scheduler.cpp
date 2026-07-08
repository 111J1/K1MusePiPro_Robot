#include "k1muse_ai_runtime/runtime_scheduler.hpp"

#include <exception>
#include <utility>

namespace k1muse_ai_runtime
{

RuntimeScheduler::RuntimeScheduler(
  ResourceGuard & guard, std::size_t capacity,
  std::chrono::milliseconds stop_timeout)
: guard_(guard), capacity_(capacity), stop_timeout_(stop_timeout)
{
  stats_.accepting = true;
}

RuntimeScheduler::~RuntimeScheduler()
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
    // InferenceJob execute functions are contractually cooperative. Destruction is the
    // final ownership barrier and joins rather than detaching or freeing live state.
    worker_.join();
  }
}

bool RuntimeScheduler::Compare::operator()(
  const InferenceJob & lhs, const InferenceJob & rhs) const
{
  if (lhs.priority != rhs.priority) {
    return static_cast<int>(lhs.priority) < static_cast<int>(rhs.priority);
  }
  return lhs.sequence > rhs.sequence;
}

bool RuntimeScheduler::submit(InferenceJob job)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool guarded_module =
      job.module == JobModule::ASR_EP ||
      job.module == JobModule::TTS_EP ||
      job.module == JobModule::VISION_EP;
    if (!guarded_module || !job_contract_valid(job) ||
      job.provider_class != canonical_provider(job.module) ||
      job.priority != canonical_priority(job.module))
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
    job.sequence = next_sequence_++;
    queue_.push(std::move(job));
    ++stats_.submitted;
    stats_.queued = queue_.size();
  }
  work_available_.notify_one();
  notify_state();
  return true;
}

void RuntimeScheduler::start()
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
    worker_ = std::thread(&RuntimeScheduler::worker_loop, this);
  }
  notify_state();
}

void RuntimeScheduler::reject_new_jobs()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.accepting = false;
  }
  notify_state();
}

bool RuntimeScheduler::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.accepting = false;
    stop_requested_ = true;
    const auto pending = queue_.size();
    while (!queue_.empty()) {
      queue_.pop();
    }
    stats_.dropped += pending;
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

std::size_t RuntimeScheduler::cancel_pending()
{
  std::size_t count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    count = queue_.size();
    while (!queue_.empty()) {
      queue_.pop();
    }
    stats_.dropped += count;
    stats_.queued = 0;
  }
  notify_state();
  return count;
}

QueueStats RuntimeScheduler::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = stats_;
  result.queued = queue_.size();
  return result;
}

bool RuntimeScheduler::has_live_thread() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

void RuntimeScheduler::set_state_callback(std::function<void()> callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_callback_ = std::move(callback);
}

void RuntimeScheduler::worker_loop()
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
      job = queue_.top();
      queue_.pop();
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
        {
          auto lease = guard_.acquire(job.id);
          if (!job_is_current(job, *token)) {
            dropped = true;
          } else {
            job.execute(*token);
            executed = true;
          }
        }
        if (executed) {
          committed = job.commit_if_current(*token);
          dropped = !committed;
        }
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

void RuntimeScheduler::notify_state()
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
