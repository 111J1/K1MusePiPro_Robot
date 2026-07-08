#include "k1muse_ai_runtime/models/wakeword_runtime.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace k1muse_ai_runtime
{

WakewordRuntime::WakewordRuntime(
  std::unique_ptr<WakewordBackend> backend,
  WakewordCallback callback)
: backend_(std::move(backend)),
  callback_(std::move(callback))
{
}

const std::string & WakewordRuntime::name() const
{
  static const std::string n{"wakeword"};
  return n;
}

const std::string & WakewordRuntime::provider() const
{
  static const std::string p{"cpu"};
  return p;
}

void WakewordRuntime::load(const CancellationToken & token, Deadline deadline)
{
  if (token.stop_requested() || std::chrono::steady_clock::now() > deadline) {
    throw std::runtime_error("wakeword load cancelled or expired");
  }
  stop_requested_.store(false);
  backend_->load();
}

void WakewordRuntime::warmup(const CancellationToken & token, Deadline deadline)
{
  if (!backend_->loaded()) {
    throw std::runtime_error("wakeword backend is not loaded");
  }
  // Poll cancellation during a brief warmup period.
  const auto warmup_end =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
  while (std::chrono::steady_clock::now() < warmup_end) {
    if (token.stop_requested() || stop_requested_.load() ||
      std::chrono::steady_clock::now() > deadline)
    {
      throw std::runtime_error("wakeword warmup cancelled");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void WakewordRuntime::request_cancel() noexcept
{
  stop_requested_.store(true);
}

bool WakewordRuntime::stop(std::chrono::milliseconds /*stop_timeout*/) noexcept
{
  stop_requested_.store(true);
  return true;
}

void WakewordRuntime::final_join() noexcept
{
  // No threads owned by this runtime.
}

void WakewordRuntime::unload() noexcept
{
  if (backend_) {
    backend_->unload();
  }
}

bool WakewordRuntime::loaded() const noexcept
{
  return backend_ && backend_->loaded();
}

void WakewordRuntime::process_audio(
  const int16_t * pcm, size_t samples,
  const std::string & trace_id, uint64_t epoch,
  const ControlSnapshot & control)
{
  if (stop_requested_.load() || !control.wakeword_enabled) {
    return;
  }
  if (!backend_ || !backend_->loaded()) {
    return;
  }

  float confidence = 0.0f;
  std::string keyword;
  if (backend_->detect(pcm, samples, confidence, keyword)) {
    if (callback_) {
      callback_(trace_id, epoch, confidence, keyword);
    }
  }
}

}  // namespace k1muse_ai_runtime
