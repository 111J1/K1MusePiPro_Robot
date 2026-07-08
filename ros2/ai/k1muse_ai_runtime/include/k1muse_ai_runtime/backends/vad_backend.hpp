#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace k1muse_ai_runtime
{

class VadBackend
{
public:
  virtual ~VadBackend() = default;

  virtual const std::string & name() const = 0;

  virtual void load() = 0;
  virtual void unload() = 0;
  virtual bool loaded() const = 0;

  /// Process one audio chunk. Returns speech probability in [0.0, 1.0].
  virtual float process(const int16_t * pcm, size_t samples) = 0;

  virtual void reset() = 0;
};

}  // namespace k1muse_ai_runtime
