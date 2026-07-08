#pragma once

#include "k1muse_ai_runtime/backends/vad_backend.hpp"

#include <string>

namespace k1muse_ai_runtime
{

/// Deterministic mock VAD backend for testing.
/// Computes RMS of the PCM samples and normalises to [0, 1].
/// The mock returns the raw normalised RMS energy directly; consumers
/// should compare the returned value against their own threshold.
class MockVadBackend : public VadBackend
{
public:
  MockVadBackend();

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  float process(const int16_t * pcm, size_t samples) override;

  void reset() override;

private:
  bool loaded_{false};
};

}  // namespace k1muse_ai_runtime
