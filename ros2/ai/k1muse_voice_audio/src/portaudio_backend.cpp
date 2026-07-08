#include "k1muse_voice_audio/portaudio_backend.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include <portaudio.h>

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

int find_device_by_name(const std::string & hint, bool is_input)
{
  int count = Pa_GetDeviceCount();
  for (int i = 0; i < count; ++i) {
    const auto * info = Pa_GetDeviceInfo(i);
    if (!info) continue;
    if (is_input && info->maxInputChannels == 0) continue;
    if (!is_input && info->maxOutputChannels == 0) continue;
    std::string name(info->name);
    if (!hint.empty() && name.find(hint) != std::string::npos) return i;
  }
  return is_input ? Pa_GetDefaultInputDevice() : Pa_GetDefaultOutputDevice();
}
}  // namespace

struct PortAudioBackend::Impl
{
  AudioConfig config;
  bool pa_initialized{false};
  bool capture_open{false};
  bool playback_open{false};

  PaStream * capture_stream{nullptr};
  PaStream * playback_stream{nullptr};
  int playback_sample_rate{0};
};

PortAudioBackend::PortAudioBackend()
  : impl_(std::make_unique<Impl>())
{
}

PortAudioBackend::~PortAudioBackend()
{
  close();
}

bool PortAudioBackend::open_capture(const AudioConfig & config, std::string * error)
{
  impl_->config = config;

  if (!impl_->pa_initialized) {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      set_error(error, std::string("Pa_Initialize: ") + Pa_GetErrorText(err));
      return false;
    }
    impl_->pa_initialized = true;
  }

  int dev_id = find_device_by_name(config.capture_device, true);
  PaStreamParameters input_params{};
  input_params.device = dev_id;
  input_params.channelCount = config.channels;
  input_params.sampleFormat = paInt16;
  input_params.suggestedLatency =
    Pa_GetDeviceInfo(dev_id)->defaultLowInputLatency;

  unsigned long frames_per_buffer =
    static_cast<unsigned long>(config.sample_rate * config.chunk_ms / 1000);

  PaError err = Pa_OpenStream(
    &impl_->capture_stream, &input_params, nullptr,
    static_cast<double>(config.sample_rate),
    frames_per_buffer, paClipOff, nullptr, nullptr);
  if (err != paNoError) {
    set_error(error, std::string("capture Pa_OpenStream: ") + Pa_GetErrorText(err));
    return false;
  }

  err = Pa_StartStream(impl_->capture_stream);
  if (err != paNoError) {
    set_error(error, std::string("capture Pa_StartStream: ") + Pa_GetErrorText(err));
    Pa_CloseStream(impl_->capture_stream);
    impl_->capture_stream = nullptr;
    return false;
  }

  impl_->capture_open = true;
  set_error(error, "");
  return true;
}

bool PortAudioBackend::open_playback(const AudioConfig & config, std::string * error)
{
  impl_->config = config;
  impl_->playback_sample_rate = config.sample_rate;

  if (!impl_->pa_initialized) {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      set_error(error, std::string("Pa_Initialize: ") + Pa_GetErrorText(err));
      return false;
    }
    impl_->pa_initialized = true;
  }

  int dev_id = find_device_by_name(config.playback_device, false);
  PaStreamParameters output_params{};
  output_params.device = dev_id;
  output_params.channelCount = config.channels;
  output_params.sampleFormat = paInt16;
  output_params.suggestedLatency =
    Pa_GetDeviceInfo(dev_id)->defaultLowOutputLatency;

  PaError err = Pa_OpenStream(
    &impl_->playback_stream, nullptr, &output_params,
    static_cast<double>(config.sample_rate),
    paFramesPerBufferUnspecified, paClipOff, nullptr, nullptr);
  if (err != paNoError) {
    set_error(error, std::string("playback Pa_OpenStream: ") + Pa_GetErrorText(err));
    return false;
  }

  err = Pa_StartStream(impl_->playback_stream);
  if (err != paNoError) {
    set_error(error, std::string("playback Pa_StartStream: ") + Pa_GetErrorText(err));
    Pa_CloseStream(impl_->playback_stream);
    impl_->playback_stream = nullptr;
    return false;
  }

  impl_->playback_open = true;
  set_error(error, "");
  return true;
}

bool PortAudioBackend::read_chunk(std::vector<int16_t> * samples, std::string * error)
{
  if (!impl_->capture_open || !impl_->capture_stream) {
    set_error(error, "capture not open");
    return false;
  }

  unsigned long frames =
    static_cast<unsigned long>(
      impl_->config.sample_rate * impl_->config.chunk_ms / 1000);

  samples->resize(frames * impl_->config.channels);
  PaError err = Pa_ReadStream(
    impl_->capture_stream, samples->data(), frames);

  if (err == paNoError) {
    set_error(error, "");
    return true;
  }
  if (err == paInputOverflowed) {
    set_error(error, "");
    return true;
  }
  set_error(error, std::string("Pa_ReadStream: ") + Pa_GetErrorText(err));
  return false;
}

bool PortAudioBackend::write_chunk(
  const std::vector<int16_t> & samples,
  int sample_rate,
  std::string * error)
{
  if (!impl_->playback_open || !impl_->playback_stream) {
    set_error(error, "playback not open");
    return false;
  }

  if (sample_rate != impl_->playback_sample_rate) {
    set_error(error, "sample rate mismatch");
    return false;
  }

  if (samples.empty()) {
    set_error(error, "");
    return true;
  }

  unsigned long frames =
    static_cast<unsigned long>(samples.size() / impl_->config.channels);

  PaError err = Pa_WriteStream(
    impl_->playback_stream, samples.data(), frames);
  if (err == paNoError || err == paOutputUnderflowed) {
    set_error(error, "");
    return true;
  }
  set_error(error, std::string("Pa_WriteStream: ") + Pa_GetErrorText(err));
  return false;
}

void PortAudioBackend::close()
{
  if (impl_->capture_stream) {
    Pa_StopStream(impl_->capture_stream);
    Pa_CloseStream(impl_->capture_stream);
    impl_->capture_stream = nullptr;
  }
  if (impl_->playback_stream) {
    Pa_StopStream(impl_->playback_stream);
    Pa_CloseStream(impl_->playback_stream);
    impl_->playback_stream = nullptr;
  }
  if (impl_->pa_initialized) {
    Pa_Terminate();
    impl_->pa_initialized = false;
  }
  impl_->capture_open = false;
  impl_->playback_open = false;
}

bool PortAudioBackend::requires_external_pacing() const
{
  return false;  // PortAudio blocking read provides natural pacing
}

bool PortAudioBackend::reconnect_capture(std::string * error)
{
  // Close broken stream
  if (impl_->capture_stream) {
    Pa_StopStream(impl_->capture_stream);
    Pa_CloseStream(impl_->capture_stream);
    impl_->capture_stream = nullptr;
  }
  impl_->capture_open = false;

  // Reopen
  return open_capture(impl_->config, error);
}

std::unique_ptr<AudioBackend> create_portaudio_audio_backend()
{
  return std::make_unique<PortAudioBackend>();
}

}  // namespace k1muse_voice_audio
