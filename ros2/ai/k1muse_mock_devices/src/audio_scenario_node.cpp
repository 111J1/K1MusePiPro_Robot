#include "k1muse_mock_devices/audio_scenario.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace k1muse_mock_devices
{

// ---------------------------------------------------------------------------
// AudioScenario implementation
// ---------------------------------------------------------------------------

AudioScenario::AudioScenario(const Config & config) : config_(config) {}

std::size_t AudioScenario::samples_per_frame() const
{
  return static_cast<std::size_t>(config_.sample_rate) * config_.frame_ms / 1000;
}

AudioScenario::FrameData AudioScenario::make_frame(
  const std::string & trace_id, uint32_t seq,
  const std::vector<int16_t> & pcm)
{
  FrameData f;
  f.trace_id = trace_id;
  f.seq = seq;
  f.sample_rate = config_.sample_rate;
  f.channels = config_.channels;
  f.encoding = "s16le";
  f.frame_ms = config_.frame_ms;
  f.pcm = pcm;
  return f;
}

std::vector<AudioScenario::FrameData> AudioScenario::generate(
  Type type, const std::string & trace_id)
{
  switch (type) {
    case Type::WakeMarker: return generate_wake_marker(trace_id);
    case Type::Speech:     return generate_speech(trace_id);
    case Type::Silence:    return generate_silence(trace_id);
    case Type::SeqGap:     return generate_seq_gap(trace_id);
    case Type::NoSpeech:   return generate_no_speech(trace_id);
  }
  return {};
}

std::vector<AudioScenario::FrameData> AudioScenario::generate_wake_marker(
  const std::string & trace_id)
{
  const auto n = samples_per_frame();
  std::vector<FrameData> frames;
  frames.reserve(config_.frames_per_scenario);
  uint32_t seq = config_.seq_start;

  // First frame: trigger sample at position 0
  std::vector<int16_t> pcm(n, 0);
  pcm[0] = config_.wake_marker_value;
  frames.push_back(make_frame(trace_id, seq++, pcm));

  // Remaining frames: silence
  for (uint32_t i = 1; i < config_.frames_per_scenario; ++i) {
    frames.push_back(make_frame(trace_id, seq++, std::vector<int16_t>(n, 0)));
  }
  return frames;
}

std::vector<AudioScenario::FrameData> AudioScenario::generate_speech(
  const std::string & trace_id)
{
  const auto n = samples_per_frame();
  std::vector<FrameData> frames;
  frames.reserve(config_.frames_per_scenario);
  uint32_t seq = config_.seq_start;

  // Generate deterministic speech-like PCM: sine wave at 440 Hz with
  // amplitude chosen so the RMS approximates speech_level.
  const double amplitude = config_.speech_level * std::sqrt(2.0);
  const double freq = 440.0;
  const double phase_step = 2.0 * M_PI * freq / config_.sample_rate;

  for (uint32_t i = 0; i < config_.frames_per_scenario; ++i) {
    std::vector<int16_t> pcm(n);
    const double phase_base = i * n * phase_step;
    for (std::size_t j = 0; j < n; ++j) {
      double val = amplitude * std::sin(phase_base + j * phase_step);
      if (val > 32767.0) val = 32767.0;
      if (val < -32768.0) val = -32768.0;
      pcm[j] = static_cast<int16_t>(val);
    }
    frames.push_back(make_frame(trace_id, seq++, pcm));
  }
  return frames;
}

std::vector<AudioScenario::FrameData> AudioScenario::generate_silence(
  const std::string & trace_id)
{
  const auto n = samples_per_frame();
  std::vector<FrameData> frames;
  frames.reserve(config_.frames_per_scenario);
  uint32_t seq = config_.seq_start;

  for (uint32_t i = 0; i < config_.frames_per_scenario; ++i) {
    frames.push_back(make_frame(trace_id, seq++, std::vector<int16_t>(n, 0)));
  }
  return frames;
}

std::vector<AudioScenario::FrameData> AudioScenario::generate_seq_gap(
  const std::string & trace_id)
{
  const auto n = samples_per_frame();
  std::vector<FrameData> frames;

  // First 10 frames: seq 1..10
  for (uint32_t i = 0; i < 10; ++i) {
    frames.push_back(make_frame(trace_id, config_.seq_start + i,
      std::vector<int16_t>(n, config_.speech_level)));
  }
  // Next 10 frames: seq 20..29 (gap of 9)
  for (uint32_t i = 0; i < 10; ++i) {
    frames.push_back(make_frame(trace_id, config_.seq_start + 19 + i,
      std::vector<int16_t>(n, config_.speech_level)));
  }
  return frames;
}

std::vector<AudioScenario::FrameData> AudioScenario::generate_no_speech(
  const std::string & trace_id)
{
  return generate_silence(trace_id);
}

}  // namespace k1muse_mock_devices
