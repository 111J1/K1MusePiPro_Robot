#pragma once

#include "k1muse_ai_runtime/backends/wakeword_backend.hpp"

#include <memory>
#include <string>

namespace k1muse_ai_runtime
{

/// Real wakeword backend using sherpa-onnx KeywordSpotter.
class RealSherpaWakewordBackend : public WakewordBackend
{
public:
  RealSherpaWakewordBackend(
    const std::string & model_dir,
    float threshold = 0.25f,
    const std::string & keywords_file = "");

  ~RealSherpaWakewordBackend() override;

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  bool detect(
    const int16_t * pcm, size_t samples,
    float & confidence, std::string & keyword) override;

  void reset() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_ai_runtime
