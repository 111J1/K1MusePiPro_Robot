#pragma once

#include <memory>
#include <string>

#include "k1muse_voice_audio/audio_backend.hpp"

namespace k1muse_voice_audio
{

/// PortAudio-based audio backend for K1.
/// Uses PortAudio for both capture and playback, compatible with
/// PipeWire (avoids raw-ALSA EBUSY).
class PortAudioBackend final : public AudioBackend
{
public:
  PortAudioBackend();
  ~PortAudioBackend() override;

  bool open_capture(const AudioConfig & config, std::string * error) override;
  bool open_playback(const AudioConfig & config, std::string * error) override;
  bool read_chunk(std::vector<int16_t> * samples, std::string * error) override;
  bool write_chunk(
    const std::vector<int16_t> & samples,
    int sample_rate,
    std::string * error) override;
  void close() override;
  bool requires_external_pacing() const override;
  bool reconnect_capture(std::string * error) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_voice_audio
