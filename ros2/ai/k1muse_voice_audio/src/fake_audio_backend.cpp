#include "k1muse_voice_audio/audio_backend.hpp"

#include <utility>

namespace k1muse_voice_audio
{

namespace
{
void set_error(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}
}  // namespace

bool FakeAudioBackend::open_capture(const AudioConfig & config, std::string * error)
{
  config_ = config;
  capture_open_ = true;
  set_error(error, "");
  return true;
}

bool FakeAudioBackend::open_playback(const AudioConfig & config, std::string * error)
{
  config_ = config;
  playback_open_ = true;
  set_error(error, "");
  return true;
}

bool FakeAudioBackend::read_chunk(std::vector<int16_t> * samples, std::string * error)
{
  if (!capture_open_) {
    set_error(error, "fake capture backend is not open");
    return false;
  }
  const auto frames = static_cast<size_t>(
    config_.sample_rate * config_.chunk_ms / 1000);
  samples->assign(frames * config_.channels, 0);
  set_error(error, "");
  return true;
}

bool FakeAudioBackend::write_chunk(
  const std::vector<int16_t> & samples,
  int /*sample_rate*/,
  std::string * error)
{
  if (!playback_open_) {
    set_error(error, "fake playback backend is not open");
    return false;
  }
  playback_chunks_.push_back(samples);
  set_error(error, "");
  return true;
}

void FakeAudioBackend::close()
{
  capture_open_ = false;
  playback_open_ = false;
}

bool FakeAudioBackend::requires_external_pacing() const
{
  return true;
}

const std::vector<std::vector<int16_t>> & FakeAudioBackend::recorded_playback_chunks() const
{
  return playback_chunks_;
}

std::unique_ptr<AudioBackend> create_fake_audio_backend()
{
  return std::make_unique<FakeAudioBackend>();
}

}  // namespace k1muse_voice_audio
