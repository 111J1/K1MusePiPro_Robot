#include "k1muse_ai_runtime/backends/mock_vad_backend.hpp"

#include <algorithm>
#include <cmath>

namespace k1muse_ai_runtime
{

MockVadBackend::MockVadBackend() = default;

const std::string & MockVadBackend::name() const
{
  static const std::string n{"mock_vad"};
  return n;
}

void MockVadBackend::load()
{
  loaded_ = true;
}

void MockVadBackend::unload()
{
  loaded_ = false;
}

bool MockVadBackend::loaded() const
{
  return loaded_;
}

float MockVadBackend::process(const int16_t * pcm, size_t samples)
{
  if (!loaded_ || pcm == nullptr || samples == 0) {
    return 0.0f;
  }

  // Compute RMS energy of the PCM samples.
  double sum_sq = 0.0;
  for (size_t i = 0; i < samples; ++i) {
    const double s = static_cast<double>(pcm[i]);
    sum_sq += s * s;
  }
  const double rms = std::sqrt(sum_sq / static_cast<double>(samples));

  // Normalise to [0, 1] where full-scale int16 (32768) maps to 1.0.
  const double normalised = std::min(rms / 32768.0, 1.0);

  return static_cast<float>(normalised);
}

void MockVadBackend::reset()
{
  // Nothing to reset beyond load state for the mock.
}

}  // namespace k1muse_ai_runtime
