#include "k1muse_ai_runtime/backends/mock_tts_backend.hpp"

#include <cmath>

namespace k1muse_ai_runtime
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kFrequencyHz = 440.0;  // A4
constexpr double kAmplitude = 8000.0;   // Well within int16 range
}  // namespace

MockTtsBackend::MockTtsBackend() = default;

const std::string & MockTtsBackend::name() const
{
  static const std::string n{"mock_tts"};
  return n;
}

void MockTtsBackend::load()
{
  loaded_ = true;
}

void MockTtsBackend::unload()
{
  loaded_ = false;
}

bool MockTtsBackend::loaded() const
{
  return loaded_;
}

void MockTtsBackend::warmup()
{
  // Nothing to do for the mock.
}

TtsBackend::Result MockTtsBackend::synthesize(
  const std::string & text, const std::string & /*voice*/)
{
  if (!loaded_) {
    return Result{false, {}, 0, 0, "", "backend not loaded"};
  }

  if (text.empty()) {
    return Result{false, {}, 0, 0, "", "empty text"};
  }

  const uint32_t num_samples =
    mock_sample_rate_ * mock_duration_ms_ / 1000;

  std::vector<int16_t> pcm(num_samples);
  for (uint32_t i = 0; i < num_samples; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(mock_sample_rate_);
    double value = kAmplitude * std::sin(2.0 * kPi * kFrequencyHz * t);
    pcm[i] = static_cast<int16_t>(value);
  }

  return Result{true, std::move(pcm), mock_sample_rate_, mock_channels_, mock_encoding_, ""};
}

void MockTtsBackend::reset()
{
  // Nothing to reset beyond load state for the mock.
}

void MockTtsBackend::set_mock_sample_rate(uint32_t sample_rate)
{
  mock_sample_rate_ = sample_rate;
}

void MockTtsBackend::set_mock_channels(uint8_t channels)
{
  mock_channels_ = channels;
}

void MockTtsBackend::set_mock_encoding(const std::string & encoding)
{
  mock_encoding_ = encoding;
}

void MockTtsBackend::set_mock_duration_ms(uint32_t duration_ms)
{
  mock_duration_ms_ = duration_ms;
}

}  // namespace k1muse_ai_runtime
