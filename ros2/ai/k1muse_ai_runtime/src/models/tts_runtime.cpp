#include "k1muse_ai_runtime/models/tts_runtime.hpp"

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <utility>

#include "k1muse_voice_msgs/msg/tts_text_request.hpp"

namespace k1muse_ai_runtime
{

// TtsStatus state constants (mirroring TtsStatus.msg).
namespace tts_state
{
constexpr uint8_t PENDING = 0;
constexpr uint8_t RUNNING = 1;
constexpr uint8_t DONE = 2;
constexpr uint8_t FAILED = 3;
constexpr uint8_t CANCELLED = 4;
}  // namespace tts_state

TTSRuntime::TTSRuntime(
  std::unique_ptr<TtsBackend> backend,
  const std::string & provider,
  StatusCallback callbacks)
: backend_(std::move(backend)),
  provider_(provider),
  callbacks_(std::move(callbacks))
{
}

const std::string & TTSRuntime::name() const
{
  static const std::string n{"tts"};
  return n;
}

const std::string & TTSRuntime::provider() const
{
  return provider_;
}

void TTSRuntime::load(const CancellationToken & token, Deadline deadline)
{
  if (token.stop_requested() || std::chrono::steady_clock::now() > deadline) {
    throw std::runtime_error("tts load cancelled or expired");
  }
  stop_requested_.store(false);
  backend_->load();
}

void TTSRuntime::warmup(const CancellationToken & token, Deadline deadline)
{
  if (!backend_->loaded()) {
    throw std::runtime_error("tts backend is not loaded");
  }
  // Poll cancellation during a brief warmup period.
  const auto warmup_end =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
  while (std::chrono::steady_clock::now() < warmup_end) {
    if (token.stop_requested() || stop_requested_.load() ||
      std::chrono::steady_clock::now() > deadline)
    {
      throw std::runtime_error("tts warmup cancelled");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void TTSRuntime::request_cancel() noexcept
{
  stop_requested_.store(true);
}

bool TTSRuntime::stop(std::chrono::milliseconds /*stop_timeout*/) noexcept
{
  stop_requested_.store(true);
  return true;
}

void TTSRuntime::final_join() noexcept
{
  // No threads owned by this runtime.
}

void TTSRuntime::unload() noexcept
{
  if (backend_) {
    backend_->unload();
  }
  clear_pending();
}

bool TTSRuntime::loaded() const noexcept
{
  return backend_ && backend_->loaded();
}

bool TTSRuntime::submit_request(
  const std::string & trace_id, const std::string & request_id,
  uint64_t epoch, const std::string & source,
  uint8_t priority, const std::string & text, const std::string & voice)
{
  std::lock_guard<std::mutex> lock(pending_mutex_);

  if (pending_.has_value()) {
    // Slot occupied: compare priority.
    if (priority >= pending_->priority) {
      // Higher or equal priority replaces: publish CANCELLED for old request.
      if (callbacks_.publish_status) {
        callbacks_.publish_status(
          pending_->trace_id, pending_->request_id,
          pending_->epoch, pending_->source,
          tts_state::CANCELLED, "cancelled", "replaced by higher priority");
      }
      // Fall through to accept new request below.
    } else {
      // Lower priority: reject new request.
      if (callbacks_.publish_status) {
        callbacks_.publish_status(
          trace_id, request_id, epoch, source,
          tts_state::FAILED, "rejected", "lower priority than pending request");
      }
      return false;
    }
  }

  // Accept the new request.
  pending_ = PendingRequest{trace_id, request_id, epoch, source, priority, text, voice};

  if (callbacks_.publish_status) {
    callbacks_.publish_status(
      trace_id, request_id, epoch, source,
      tts_state::PENDING, "pending", "");
  }

  return true;
}

bool TTSRuntime::process_pending(
  const std::function<bool(uint64_t)> & epoch_is_current,
  const std::function<bool(JobModule)> & /*module_enabled*/,
  const std::function<bool()> & /*cancel_predicate*/,
  std::chrono::steady_clock::time_point /*deadline*/,
  uint64_t /*current_epoch*/, bool tts_enabled)
{
  if (stop_requested_.load()) {
    return false;
  }

  PendingRequest req;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!pending_.has_value()) {
      return false;
    }
    // High-priority requests (reminder / system) bypass the tts_enabled
    // gate so due reminders and system alerts can play even when the
    // Supervisor is in IDLE state.
    if (!tts_enabled && pending_->priority < k1muse_voice_msgs::msg::TtsTextRequest::PRIORITY_REMINDER) {
      return false;
    }
    req = std::move(pending_.value());
    pending_.reset();
  }

  // Validate text is non-empty.
  if (req.text.empty()) {
    if (callbacks_.publish_status) {
      callbacks_.publish_status(
        req.trace_id, req.request_id, req.epoch, req.source,
        tts_state::FAILED, "failed", "empty text");
    }
    return true;
  }

  // Run synthesis.
  PcmResult result = backend_->synthesize(req.text, req.voice);

  // Epoch gating: if stale, drop (new wakeword arrived during synthesis).
  if (!epoch_is_current(req.epoch)) {
    fprintf(stderr,
      "[tts_runtime] dropping stale TTS result for epoch=%lu "
      "(new session started during synthesis)\n",
      static_cast<unsigned long>(req.epoch));
    return true;
  }

  if (result.success) {
    // Publish DONE status.
    if (callbacks_.publish_status) {
      callbacks_.publish_status(
        req.trace_id, req.request_id, req.epoch, req.source,
        tts_state::DONE, "done", "");
    }
    // Publish play request with PCM.
    if (callbacks_.publish_play) {
      callbacks_.publish_play(
        req.trace_id, req.request_id, req.epoch, req.source, result);
    }
  } else {
    // Publish FAILED status.
    if (callbacks_.publish_status) {
      callbacks_.publish_status(
        req.trace_id, req.request_id, req.epoch, req.source,
        tts_state::FAILED, "failed", result.reason);
    }
  }

  return true;
}

void TTSRuntime::clear_pending()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_.reset();
}

bool TTSRuntime::has_pending() const
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  return pending_.has_value();
}

}  // namespace k1muse_ai_runtime
