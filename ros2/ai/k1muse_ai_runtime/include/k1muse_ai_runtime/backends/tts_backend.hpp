#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

class TtsBackend
{
public:
  struct Result
  {
    bool success{false};
    std::vector<int16_t> pcm_s16le;
    uint32_t sample_rate{0};
    uint8_t channels{0};
    std::string encoding;
    std::string reason;
  };

  virtual ~TtsBackend() = default;

  virtual const std::string & name() const = 0;

  virtual void load() = 0;
  virtual void unload() = 0;
  virtual bool loaded() const = 0;

  virtual void warmup() = 0;

  /// Synthesize text to PCM audio.
  virtual Result synthesize(const std::string & text, const std::string & voice) = 0;

  virtual void reset() = 0;
};

}  // namespace k1muse_ai_runtime
