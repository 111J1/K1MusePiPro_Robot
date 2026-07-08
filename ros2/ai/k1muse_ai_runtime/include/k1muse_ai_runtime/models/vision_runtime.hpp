#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "k1muse_ai_runtime/backends/vision_backend.hpp"
#include "k1muse_ai_runtime/model_runtime.hpp"
#include "k1muse_ai_runtime/vision/latest_frame_buffer.hpp"

namespace k1muse_ai_runtime
{

class VisionRuntime final : public ModelRuntime
{
public:
  using DetectionData = VisionBackend::Detection;
  using FrameResult = VisionBackend::FrameResult;

  using ResultCallback = std::function<void(
    const std::string & trace_id, uint64_t epoch,
    uint32_t image_width, uint32_t image_height,
    std::vector<DetectionData> detections)>;

  VisionRuntime(
    std::unique_ptr<VisionBackend> backend,
    const std::string & provider,
    ResultCallback result_callback);

  // ModelRuntime interface
  const std::string & name() const override;
  const std::string & provider() const override;
  void load(const CancellationToken & token, Deadline deadline) override;
  void warmup(const CancellationToken & token, Deadline deadline) override;
  void request_cancel() noexcept override;
  bool stop(std::chrono::milliseconds stop_timeout) noexcept override;
  void final_join() noexcept override;
  void unload() noexcept override;
  bool loaded() const noexcept override;

  /// Store frame metadata (and optional image data) in the buffer.
  /// Does NOT run inference.
  /// @param step Row stride in bytes (0 means tightly packed).
  void put_frame(const std::string & frame_id,
                 uint32_t width, uint32_t height,
                 const std::string & encoding,
                 const std::string & trace_id, uint64_t epoch,
                 const uint8_t * data = nullptr, size_t data_size = 0,
                 uint32_t step = 0);

  /// Process the latest frame from the buffer.
  /// Returns true if a frame was processed, false if no frame available.
  bool process_latest_frame(
    const std::function<bool(uint64_t)> & epoch_is_current,
    const std::function<bool(JobModule)> & module_enabled,
    const std::function<bool()> & cancel_predicate,
    std::chrono::steady_clock::time_point deadline,
    uint64_t current_epoch);

  /// Clear the buffer (when vision is disabled).
  void clear_buffer();

private:
  struct FrameData
  {
    std::string frame_id;
    uint32_t width{0};
    uint32_t height{0};
    std::string encoding;
    std::string trace_id;
    uint64_t epoch{0};
    uint32_t step{0};           // row stride in bytes (0 = tightly packed)
    std::vector<uint8_t> data;  // image pixel data
  };

  std::unique_ptr<VisionBackend> backend_;
  std::string provider_;
  ResultCallback result_callback_;
  LatestFrameBuffer<FrameData> buffer_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> processing_{false};
};

}  // namespace k1muse_ai_runtime
