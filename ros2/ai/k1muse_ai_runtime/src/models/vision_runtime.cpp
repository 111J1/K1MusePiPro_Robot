#include "k1muse_ai_runtime/models/vision_runtime.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace k1muse_ai_runtime
{

VisionRuntime::VisionRuntime(
  std::unique_ptr<VisionBackend> backend,
  const std::string & provider,
  ResultCallback result_callback)
: backend_(std::move(backend)),
  provider_(provider),
  result_callback_(std::move(result_callback))
{
}

const std::string & VisionRuntime::name() const
{
  static const std::string n{"vision"};
  return n;
}

const std::string & VisionRuntime::provider() const
{
  return provider_;
}

void VisionRuntime::load(const CancellationToken & token, Deadline deadline)
{
  if (token.stop_requested() || std::chrono::steady_clock::now() > deadline) {
    throw std::runtime_error("vision load cancelled or expired");
  }
  stop_requested_.store(false);
  backend_->load();
}

void VisionRuntime::warmup(const CancellationToken & token, Deadline deadline)
{
  if (!backend_->loaded()) {
    throw std::runtime_error("vision backend is not loaded");
  }
  // Poll cancellation during a brief warmup period.
  const auto warmup_end =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
  while (std::chrono::steady_clock::now() < warmup_end) {
    if (token.stop_requested() || stop_requested_.load() ||
      std::chrono::steady_clock::now() > deadline)
    {
      throw std::runtime_error("vision warmup cancelled");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void VisionRuntime::request_cancel() noexcept
{
  stop_requested_.store(true);
}

bool VisionRuntime::stop(std::chrono::milliseconds /*stop_timeout*/) noexcept
{
  stop_requested_.store(true);
  return true;
}

void VisionRuntime::final_join() noexcept
{
  // No threads owned by this runtime.
}

void VisionRuntime::unload() noexcept
{
  if (backend_) {
    backend_->unload();
  }
  buffer_.clear();
}

bool VisionRuntime::loaded() const noexcept
{
  return backend_ && backend_->loaded();
}

void VisionRuntime::put_frame(
  const std::string & frame_id,
  uint32_t width, uint32_t height,
  const std::string & encoding,
  const std::string & trace_id, uint64_t epoch,
  const uint8_t * data, size_t data_size,
  uint32_t step)
{
  FrameData fd;
  fd.frame_id = frame_id;
  fd.width = width;
  fd.height = height;
  fd.encoding = encoding;
  fd.trace_id = trace_id;
  fd.epoch = epoch;
  fd.step = step;
  if (data && data_size > 0) {
    fd.data.assign(data, data + data_size);
  }
  buffer_.put(std::move(fd));
}

bool VisionRuntime::process_latest_frame(
  const std::function<bool(uint64_t)> & epoch_is_current,
  const std::function<bool(JobModule)> & module_enabled,
  const std::function<bool()> & cancel_predicate,
  std::chrono::steady_clock::time_point deadline,
  uint64_t current_epoch)
{
  if (stop_requested_.load()) {
    return false;
  }

  auto entry = buffer_.get();
  if (!entry.has_value()) {
    return false;
  }

  const auto & frame = entry->frame;
  const uint64_t frame_generation = entry->generation;

  // Pre-inference gating checks.
  if (!epoch_is_current(frame.epoch)) {
    return false;
  }
  if (!module_enabled(JobModule::VISION_EP) &&
    !module_enabled(JobModule::VISION_CPU))
  {
    return false;
  }
  if (cancel_predicate()) {
    return false;
  }
  if (std::chrono::steady_clock::now() > deadline) {
    return false;
  }

  // Run inference.
  processing_.store(true);
  FrameResult result = backend_->infer(
    frame.frame_id, frame.width, frame.height, frame.encoding,
    frame.data.empty() ? nullptr : frame.data.data(), frame.data.size(),
    frame.step);
  processing_.store(false);

  // Check if a newer frame arrived during inference.
  if (buffer_.generation() != frame_generation) {
    // A newer frame superseded this one; skip publishing.
    return true;
  }

  // Re-check gates after inference.
  if (stop_requested_.load()) {
    return false;
  }
  if (!epoch_is_current(frame.epoch)) {
    return false;
  }
  if (!module_enabled(JobModule::VISION_EP) &&
    !module_enabled(JobModule::VISION_CPU))
  {
    return false;
  }
  if (cancel_predicate()) {
    return false;
  }

  if (!result.success) {
    // Backend failed; do not invoke callback.
    return true;
  }

  if (result_callback_) {
    result_callback_(
      frame.trace_id, frame.epoch,
      result.image_width, result.image_height,
      std::move(result.detections));
  }

  return true;
}

void VisionRuntime::clear_buffer()
{
  buffer_.clear();
}

}  // namespace k1muse_ai_runtime
