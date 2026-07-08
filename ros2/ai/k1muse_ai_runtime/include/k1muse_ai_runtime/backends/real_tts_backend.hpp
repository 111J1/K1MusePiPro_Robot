#pragma once

#include "k1muse_ai_runtime/backends/tts_backend.hpp"

#include <memory>
#include <string>

namespace k1muse_ai_runtime
{

/// Real TTS backend using SpacemiT TtsEngine.
class RealTtsBackend : public TtsBackend
{
public:
  RealTtsBackend(
    const std::string & model_dir,
    const std::string & backend_type = "matcha_zh_en",
    const std::string & provider = "auto");

  ~RealTtsBackend() override;

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  void warmup() override;

  Result synthesize(const std::string & text, const std::string & voice) override;

  void reset() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_ai_runtime
