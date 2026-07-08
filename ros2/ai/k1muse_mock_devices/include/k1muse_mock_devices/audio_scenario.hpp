#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_mock_devices
{

/// Deterministic audio scenario generator for testing.
/// Produces PCM frame data with known patterns so downstream consumers
/// (wakeword, VAD, ASR) can be verified without real audio hardware.
class AudioScenario
{
public:
  enum class Type { WakeMarker, Speech, Silence, SeqGap, NoSpeech };

  struct Config
  {
    uint32_t sample_rate = 16000;
    uint16_t frame_ms = 20;
    uint8_t channels = 1;
    uint32_t frames_per_scenario = 50;     // 1 second of audio at 20ms
    int16_t wake_marker_value = 0x7FFF;    // trigger sample for wakeword
    int16_t speech_level = 5000;           // RMS level for speech
    uint32_t seq_start = 1;
  };

  struct FrameData
  {
    std::string trace_id;
    uint32_t seq;
    uint32_t sample_rate;
    uint8_t channels;
    std::string encoding;
    uint16_t frame_ms;
    std::vector<int16_t> pcm;
  };

  explicit AudioScenario(const Config & config);

  /// Generate frames for a scenario type.
  /// \param type  The scenario to generate.
  /// \param trace_id  Trace identifier attached to every frame.
  /// \return Ordered vector of FrameData.
  std::vector<FrameData> generate(Type type, const std::string & trace_id);

private:
  Config config_;

  std::size_t samples_per_frame() const;

  std::vector<FrameData> generate_wake_marker(const std::string & trace_id);
  std::vector<FrameData> generate_speech(const std::string & trace_id);
  std::vector<FrameData> generate_silence(const std::string & trace_id);
  std::vector<FrameData> generate_seq_gap(const std::string & trace_id);
  std::vector<FrameData> generate_no_speech(const std::string & trace_id);

  FrameData make_frame(
    const std::string & trace_id, uint32_t seq,
    const std::vector<int16_t> & pcm);
};

}  // namespace k1muse_mock_devices
