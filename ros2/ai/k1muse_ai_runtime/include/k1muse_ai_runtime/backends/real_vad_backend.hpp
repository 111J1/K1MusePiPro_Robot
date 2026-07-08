#pragma once

#include "k1muse_ai_runtime/backends/vad_backend.hpp"

#include <memory>
#include <string>

namespace k1muse_ai_runtime
{

/// Real VAD backend using SpacemiT VadEngine.
class RealVadBackend : public VadBackend
{
public:
  RealVadBackend(
    const std::string & model_dir,
    float threshold = 0.5f);

  ~RealVadBackend() override;

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  float process(const int16_t * pcm, size_t samples) override;

  void reset() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_ai_runtime
