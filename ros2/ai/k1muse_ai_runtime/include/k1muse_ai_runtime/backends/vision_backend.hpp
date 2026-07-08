#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

class VisionBackend
{
public:
  struct Detection
  {
    std::string detection_id;
    std::string class_name;
    float score{0.0f};
    uint32_t x{0};
    uint32_t y{0};
    uint32_t width{0};
    uint32_t height{0};
  };

  struct FrameResult
  {
    bool success{false};
    uint32_t image_width{0};
    uint32_t image_height{0};
    std::vector<Detection> detections;
    std::string reason;
  };

  virtual ~VisionBackend() = default;

  virtual const std::string & name() const = 0;

  virtual void load() = 0;
  virtual void unload() = 0;
  virtual bool loaded() const = 0;

  virtual void warmup() = 0;

  /// Infer on image metadata (no actual pixel data needed for mock).
  /// Convenience overload that calls the data-accepting version with nullptr.
  FrameResult infer(
    const std::string & frame_id,
    uint32_t width, uint32_t height,
    const std::string & encoding)
  {
    return infer(frame_id, width, height, encoding, nullptr, 0, 0);
  }

  /// Infer on image data. Real backends use the pixel data; mock backends
  /// may ignore it.
  /// @param step Row stride in bytes (0 means tightly packed = width * channels).
  virtual FrameResult infer(
    const std::string & frame_id,
    uint32_t width, uint32_t height,
    const std::string & encoding,
    const uint8_t * data, size_t data_size,
    uint32_t step) = 0;

  virtual void reset() = 0;
};

}  // namespace k1muse_ai_runtime
