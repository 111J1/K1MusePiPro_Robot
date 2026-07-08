#pragma once

#include "k1muse_ai_runtime/backends/asr_backend.hpp"

#include <memory>
#include <string>

namespace k1muse_ai_runtime
{

/// Real ASR backend using SpacemiT AsrEngine (SenseVoice).
class RealSenseVoiceAsrBackend : public AsrBackend
{
public:
  RealSenseVoiceAsrBackend(
    const std::string & model_dir,
    const std::string & language = "auto",
    const std::string & provider = "spacemit");

  ~RealSenseVoiceAsrBackend() override;

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  Result transcribe(const std::vector<int16_t> & pcm, uint32_t sample_rate) override;

  void reset() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_ai_runtime
