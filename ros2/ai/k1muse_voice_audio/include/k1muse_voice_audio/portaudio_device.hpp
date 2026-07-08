#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "k1muse_voice_audio/audio_device.hpp"

namespace k1muse_voice_audio
{

/// PortAudio-based audio device for ALSA playback on K1.
///
/// Uses the SpacemiT AI SDK PortAudio wrapper (libportaudio).
/// Playback is synchronous (blocks until the PCM buffer is consumed).
class PortAudioDevice : public AudioDevice
{
public:
  struct Config
  {
    std::string playback_device{};     // ALSA device name e.g. "plughw:1,0"
    std::string preset_wake_ack_path;  // path to wozai.wav
    uint32_t sample_rate = 16000;
    uint8_t channels = 1;
    uint32_t frames_per_buffer = 320;  // 20ms @ 16kHz
  };

  explicit PortAudioDevice(Config config);
  ~PortAudioDevice() override;

  const std::string & name() const override;
  void load() override;
  void unload() override;
  bool loaded() const override;

  PlaybackResult play_pcm(
    const std::vector<int16_t> & pcm, uint32_t sample_rate,
    uint8_t channels, const std::string & encoding) override;

  PlaybackResult play_preset(const std::string & preset_name) override;

  void stop() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_voice_audio
