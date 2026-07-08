#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace k1muse_voice_audio
{

struct AudioConfig
{
  std::string backend{"fake"};
  int sample_rate{16000};
  int channels{1};
  int chunk_ms{20};
  std::string capture_device;
  std::string playback_device;
};

class AudioBackend
{
public:
  virtual ~AudioBackend() = default;

  virtual bool open_capture(const AudioConfig & config, std::string * error) = 0;
  virtual bool open_playback(const AudioConfig & config, std::string * error) = 0;
  virtual bool read_chunk(std::vector<int16_t> * samples, std::string * error) = 0;
  virtual bool write_chunk(
    const std::vector<int16_t> & samples,
    int sample_rate,
    std::string * error) = 0;
  virtual void close() = 0;
  virtual bool requires_external_pacing() const { return false; }

  /// Attempt to reconnect the capture device after a failure.
  /// Default implementation returns false (reconnection not supported).
  virtual bool reconnect_capture(std::string * error)
  {
    (void)error;
    return false;
  }
};

class FakeAudioBackend final : public AudioBackend
{
public:
  bool open_capture(const AudioConfig & config, std::string * error) override;
  bool open_playback(const AudioConfig & config, std::string * error) override;
  bool read_chunk(std::vector<int16_t> * samples, std::string * error) override;
  bool write_chunk(
    const std::vector<int16_t> & samples,
    int sample_rate,
    std::string * error) override;
  void close() override;
  bool requires_external_pacing() const override;

  const std::vector<std::vector<int16_t>> & recorded_playback_chunks() const;

private:
  AudioConfig config_;
  bool capture_open_{false};
  bool playback_open_{false};
  std::vector<std::vector<int16_t>> playback_chunks_;
};

std::unique_ptr<AudioBackend> create_fake_audio_backend();
std::unique_ptr<AudioBackend> create_alsa_audio_backend();
std::unique_ptr<AudioBackend> create_portaudio_audio_backend();

}  // namespace k1muse_voice_audio
