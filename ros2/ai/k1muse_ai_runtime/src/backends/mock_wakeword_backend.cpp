#include "k1muse_ai_runtime/backends/mock_wakeword_backend.hpp"

namespace k1muse_ai_runtime
{

MockWakewordBackend::MockWakewordBackend() = default;

const std::string & MockWakewordBackend::name() const
{
  static const std::string n{"mock_wakeword"};
  return n;
}

void MockWakewordBackend::load()
{
  loaded_ = true;
}

void MockWakewordBackend::unload()
{
  loaded_ = false;
}

bool MockWakewordBackend::loaded() const
{
  return loaded_;
}

bool MockWakewordBackend::detect(
  const int16_t * pcm, size_t samples,
  float & confidence, std::string & keyword)
{
  if (!loaded_ || pcm == nullptr || samples == 0) {
    return false;
  }

  for (size_t i = 0; i < samples; ++i) {
    if (pcm[i] == trigger_sample_value_) {
      confidence = confidence_;
      keyword = keyword_;
      return true;
    }
  }
  return false;
}

void MockWakewordBackend::reset()
{
  // Nothing to reset beyond load state for the mock.
}

void MockWakewordBackend::set_trigger_sample_value(int16_t value)
{
  trigger_sample_value_ = value;
}

void MockWakewordBackend::set_confidence(float confidence)
{
  confidence_ = confidence;
}

void MockWakewordBackend::set_keyword(const std::string & keyword)
{
  keyword_ = keyword;
}

}  // namespace k1muse_ai_runtime
