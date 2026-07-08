#pragma once

#include "k1muse_ai_runtime/backends/vision_backend.hpp"

#include <cstdint>
#include <string>

namespace k1muse_ai_runtime
{

/// Deterministic mock fire/smoke detection backend.
/// Generates fire and/or smoke detections with stable IDs
/// simulating yolov8_fire output.
class MockFireBackend : public VisionBackend
{
public:
  MockFireBackend();

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

  /// If true, generate a fire detection.
  void set_fire_present(bool present);
  /// If true, generate a smoke detection.
  void set_smoke_present(bool present);
  void set_mock_score(float score);
  void set_mock_delay_ms(int ms);

private:
  bool loaded_{false};
  bool fire_present_{true};
  bool smoke_present_{false};
  float mock_score_{0.9f};
  int mock_delay_ms_{0};
};

}  // namespace k1muse_ai_runtime
