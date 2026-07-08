#pragma once

#include "k1muse_ai_runtime/backends/vision_backend.hpp"

#include <cstdint>
#include <string>

namespace k1muse_ai_runtime
{

/// Deterministic mock vision backend for testing.
/// Generates a configurable number of detections with stable IDs.
class MockVisionBackend : public VisionBackend
{
public:
  MockVisionBackend();

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  void warmup() override;

  FrameResult infer(
    const std::string & frame_id,
    uint32_t width, uint32_t height,
    const std::string & encoding,
    const uint8_t * data, size_t data_size,
    uint32_t step) override;

  void reset() override;

  // Configuration setters (for test customization)
  void set_mock_detection_count(uint32_t count);
  void set_mock_detection_class(const std::string & class_name);
  void set_mock_detection_score(float score);

private:
  bool loaded_{false};
  uint32_t mock_detection_count_{1};
  std::string mock_detection_class_{"object"};
  float mock_detection_score_{0.9f};
};

}  // namespace k1muse_ai_runtime
