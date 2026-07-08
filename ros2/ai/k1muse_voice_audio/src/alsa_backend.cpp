#include "k1muse_voice_audio/audio_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <alsa/asoundlib.h>

namespace k1muse_voice_audio
{
namespace
{

std::string alsa_error(const std::string & prefix, int rc)
{
  return prefix + ": " + snd_strerror(rc);
}

int frames_per_chunk(const AudioConfig & config)
{
  return std::max(1, static_cast<int>(std::lround(
    static_cast<double>(config.sample_rate) * static_cast<double>(config.chunk_ms) / 1000.0)));
}

bool configure_stream(snd_pcm_t * handle, const AudioConfig & config, std::string * error)
{
  snd_pcm_hw_params_t * params = nullptr;
  snd_pcm_hw_params_alloca(&params);

  int rc = snd_pcm_hw_params_any(handle, params);
  if (rc < 0) {
    if (error) *error = alsa_error("hw_params_any failed", rc);
    return false;
  }

  rc = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (rc < 0) {
    if (error) *error = alsa_error("set_access RW_INTERLEAVED failed", rc);
    return false;
  }

  rc = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
  if (rc < 0) {
    if (error) *error = alsa_error("set_format S16_LE failed", rc);
    return false;
  }

  rc = snd_pcm_hw_params_set_channels(handle, params, static_cast<unsigned int>(config.channels));
  if (rc < 0) {
    if (error) *error = alsa_error("set_channels failed", rc);
    return false;
  }

  unsigned int rate = static_cast<unsigned int>(config.sample_rate);
  rc = snd_pcm_hw_params_set_rate_near(handle, params, &rate, nullptr);
  if (rc < 0) {
    if (error) *error = alsa_error("set_rate_near failed", rc);
    return false;
  }
  if (static_cast<int>(rate) != config.sample_rate) {
    if (error) {
      *error = "ALSA device did not accept requested sample rate " +
        std::to_string(config.sample_rate) + "Hz, got " + std::to_string(rate) + "Hz";
    }
    return false;
  }

  snd_pcm_uframes_t period_frames = static_cast<snd_pcm_uframes_t>(frames_per_chunk(config));
  rc = snd_pcm_hw_params_set_period_size_near(handle, params, &period_frames, nullptr);
  if (rc < 0) {
    if (error) *error = alsa_error("set_period_size_near failed", rc);
    return false;
  }

  snd_pcm_uframes_t buffer_frames = std::max<snd_pcm_uframes_t>(period_frames * 4, period_frames);
  snd_pcm_hw_params_set_buffer_size_near(handle, params, &buffer_frames);

  rc = snd_pcm_hw_params(handle, params);
  if (rc < 0) {
    if (error) *error = alsa_error("hw_params commit failed", rc);
    return false;
  }

  rc = snd_pcm_prepare(handle);
  if (rc < 0) {
    if (error) *error = alsa_error("prepare failed", rc);
    return false;
  }

  return true;
}

void close_stream(snd_pcm_t *& handle, bool drain)
{
  if (!handle) return;
  if (drain) {
    snd_pcm_drain(handle);
  } else {
    snd_pcm_drop(handle);
  }
  snd_pcm_close(handle);
  handle = nullptr;
}

}  // namespace

class AlsaAudioBackend final : public AudioBackend
{
public:
  ~AlsaAudioBackend() override { close(); }

  bool open_capture(const AudioConfig & config, std::string * error) override
  {
    config_ = config;
    if (config.capture_device.empty()) {
      if (error) *error = "capture_device is required for ALSA backend";
      return false;
    }

    close_stream(capture_handle_, false);
    capture_open_ = false;
    int rc = snd_pcm_open(
      &capture_handle_, config.capture_device.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {
      capture_handle_ = nullptr;
      if (error) *error = alsa_error("open capture " + config.capture_device + " failed", rc);
      return false;
    }

    if (!configure_stream(capture_handle_, config, error)) {
      close_stream(capture_handle_, false);
      return false;
    }

    capture_open_ = true;
    return true;
  }

  bool open_playback(const AudioConfig & config, std::string * error) override
  {
    config_ = config;
    if (config.playback_device.empty()) {
      if (error) *error = "playback_device is required for ALSA backend";
      return false;
    }

    close_stream(playback_handle_, true);
    playback_open_ = false;
    int rc = snd_pcm_open(
      &playback_handle_, config.playback_device.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
      playback_handle_ = nullptr;
      if (error) *error = alsa_error("open playback " + config.playback_device + " failed", rc);
      return false;
    }

    if (!configure_stream(playback_handle_, config, error)) {
      close_stream(playback_handle_, false);
      return false;
    }

    playback_open_ = true;
    return true;
  }

  bool read_chunk(std::vector<int16_t> * samples, std::string * error) override
  {
    if (!capture_handle_ || !capture_open_) {
      if (error) *error = "capture stream is not open";
      return false;
    }
    if (!samples) {
      if (error) *error = "samples output is null";
      return false;
    }

    const int frames = frames_per_chunk(config_);
    samples->assign(static_cast<size_t>(frames * config_.channels), 0);

    const snd_pcm_sframes_t rc = snd_pcm_readi(capture_handle_, samples->data(), frames);
    if (rc < 0) {
      const int recovered = snd_pcm_recover(capture_handle_, static_cast<int>(rc), 1);
      if (error) {
        *error = recovered < 0 ?
          alsa_error("capture read failed", recovered) :
          alsa_error("capture read recovered after failure", static_cast<int>(rc));
      }
      return false;
    }
    if (rc == 0) {
      if (error) *error = "capture read returned 0 frames";
      return false;
    }
    if (rc < frames) {
      samples->resize(static_cast<size_t>(rc * config_.channels));
    }
    return true;
  }

  bool write_chunk(
    const std::vector<int16_t> & samples,
    int sample_rate,
    std::string * error) override
  {
    if (!playback_handle_ || !playback_open_) {
      if (error) *error = "playback stream is not open";
      return false;
    }
    if (sample_rate != config_.sample_rate) {
      if (error) {
        *error = "playback sample rate mismatch: request=" + std::to_string(sample_rate) +
          " backend=" + std::to_string(config_.sample_rate);
      }
      return false;
    }
    if (config_.channels <= 0 || samples.size() % static_cast<size_t>(config_.channels) != 0) {
      if (error) *error = "PCM sample count is not divisible by channel count";
      return false;
    }

    const int channels = config_.channels;
    snd_pcm_sframes_t frames_remaining =
      static_cast<snd_pcm_sframes_t>(samples.size() / static_cast<size_t>(channels));
    const int16_t * cursor = samples.data();

    while (frames_remaining > 0) {
      const snd_pcm_sframes_t written =
        snd_pcm_writei(playback_handle_, cursor, frames_remaining);
      if (written < 0) {
        const int recovered = snd_pcm_recover(playback_handle_, static_cast<int>(written), 1);
        if (recovered < 0) {
          if (error) *error = alsa_error("playback write failed", recovered);
          return false;
        }
        continue;
      }
      if (written == 0) {
        if (error) *error = "playback write returned 0 frames";
        return false;
      }
      cursor += written * channels;
      frames_remaining -= written;
    }

    snd_pcm_drain(playback_handle_);
    snd_pcm_prepare(playback_handle_);
    return true;
  }

  void close() override
  {
    capture_open_ = false;
    playback_open_ = false;
    close_stream(capture_handle_, false);
    close_stream(playback_handle_, true);
  }

  bool reconnect_capture(std::string * error) override
  {
    capture_open_ = false;
    close_stream(capture_handle_, false);
    return open_capture(config_, error);
  }

private:
  AudioConfig config_;
  snd_pcm_t * capture_handle_{nullptr};
  snd_pcm_t * playback_handle_{nullptr};
  bool capture_open_{false};
  bool playback_open_{false};
};

std::unique_ptr<AudioBackend> create_alsa_audio_backend()
{
  return std::make_unique<AlsaAudioBackend>();
}

}  // namespace k1muse_voice_audio
