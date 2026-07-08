#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace k1muse_ai_runtime
{

class WakewordBackend
{
public:
  virtual ~WakewordBackend() = default;

  virtual const std::string & name() const = 0;

  virtual void load() = 0;
  virtual void unload() = 0;
  virtual bool loaded() const = 0;

  /// Process one audio chunk (PCM samples). Returns true if wakeword detected.
  /// On detection, \p confidence is set to [0.0, 1.0] and \p keyword to the
  /// matched keyword string.
  virtual bool detect(
    const int16_t * pcm, size_t samples,
    float & confidence, std::string & keyword) = 0;

  virtual void reset() = 0;
};

}  // namespace k1muse_ai_runtime
