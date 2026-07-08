#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

class AsrBackend
{
public:
  struct Result
  {
    bool success{false};
    std::string text;
    float confidence{0.0f};
    std::string language;
    std::string reason;
  };

  virtual ~AsrBackend() = default;

  virtual const std::string & name() const = 0;

  virtual void load() = 0;
  virtual void unload() = 0;
  virtual bool loaded() const = 0;

  /// Transcribe a complete speech segment. Returns an AsrBackend::Result.
  virtual Result transcribe(const std::vector<int16_t> & pcm, uint32_t sample_rate) = 0;

  virtual void reset() = 0;
};

}  // namespace k1muse_ai_runtime
