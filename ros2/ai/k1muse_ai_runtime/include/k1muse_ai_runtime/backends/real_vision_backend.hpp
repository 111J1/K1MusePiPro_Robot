#pragma once

#include "k1muse_ai_runtime/backends/vision_backend.hpp"

#include <memory>
#include <string>

namespace k1muse_ai_runtime
{

/// Real vision backend using SpacemiT VisionService.
class RealVisionBackend : public VisionBackend
{
public:
  RealVisionBackend(
    const std::string & config_path,
    const std::string & model_path = "",
    const std::string & labels_path = "",
    float conf_threshold = -1.0f,
    float iou_threshold = -1.0f);

  ~RealVisionBackend() override;

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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_ai_runtime
