#include "k1muse_voice_audio/portaudio_device.hpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include <portaudio.h>

namespace k1muse_voice_audio
{

struct PortAudioDevice::Impl
{
  Config config;
  bool pa_initialized{false};
  bool stream_open{false};
  std::atomic<bool> stop_requested{false};

  PaStream * stream{nullptr};

  explicit Impl(Config cfg) : config(std::move(cfg)) {}
};

PortAudioDevice::PortAudioDevice(Config config)
  : impl_(std::make_unique<Impl>(std::move(config)))
{
}

PortAudioDevice::~PortAudioDevice()
{
  unload();
}

const std::string & PortAudioDevice::name() const
{
  static const std::string n{"portaudio"};
  return n;
}

void PortAudioDevice::load()
{
  if (impl_->pa_initialized) return;

  PaError err = Pa_Initialize();
  if (err != paNoError) {
    std::cerr << "[portaudio] Pa_Initialize failed: "
              << Pa_GetErrorText(err) << std::endl;
    return;
  }
  impl_->pa_initialized = true;
}

void PortAudioDevice::unload()
{
  stop();

  if (impl_->pa_initialized) {
    Pa_Terminate();
    impl_->pa_initialized = false;
  }
}

bool PortAudioDevice::loaded() const
{
  return impl_->pa_initialized;
}

AudioDevice::PlaybackResult PortAudioDevice::play_pcm(
  const std::vector<int16_t> & pcm, uint32_t sample_rate,
  uint8_t channels, const std::string & /*encoding*/)
{
  PlaybackResult result;

  if (!impl_->pa_initialized) {
    result.reason = "PortAudio not initialized";
    return result;
  }

  if (pcm.empty()) {
    result.reason = "empty PCM buffer";
    return result;
  }

  impl_->stop_requested = false;

  std::cout << "[portaudio] Playback: " << pcm.size() / channels
            << " samples, " << sample_rate << "Hz, "
            << static_cast<int>(channels) << "ch" << std::endl;

  PaStreamParameters output_params{};
  output_params.device = Pa_GetDefaultOutputDevice();
  output_params.channelCount = channels;
  output_params.sampleFormat = paInt16;
  output_params.suggestedLatency =
      Pa_GetDeviceInfo(output_params.device)->defaultLowOutputLatency;

  PaStream* stream = nullptr;
  PaError err = Pa_OpenStream(&stream, nullptr, &output_params,
                              static_cast<double>(sample_rate),
                              paFramesPerBufferUnspecified,
                              paClipOff, nullptr, nullptr);
  if (err != paNoError) {
    result.reason = std::string("Pa_OpenStream: ") + Pa_GetErrorText(err);
    return result;
  }

  err = Pa_StartStream(stream);
  if (err != paNoError) {
    result.reason = std::string("Pa_StartStream: ") + Pa_GetErrorText(err);
    Pa_CloseStream(stream);
    return result;
  }

  auto start = std::chrono::steady_clock::now();
  unsigned long frames_to_write = pcm.size() / channels;
  const int16_t* buf = pcm.data();

  while (frames_to_write > 0 && !impl_->stop_requested) {
    // Wait for stream to be ready, then write what fits.
    Pa_Sleep(10);  // let stream consume previous chunk
    signed long available = Pa_GetStreamWriteAvailable(stream);
    if (available < 0) {
      result.reason = std::string("Pa_GetStreamWriteAvailable: ") + Pa_GetErrorText(available);
      break;
    }
    unsigned long chunk = static_cast<unsigned long>(available);
    if (chunk > frames_to_write) chunk = frames_to_write;
    if (chunk == 0) continue;

    err = Pa_WriteStream(stream, buf, chunk);
    if (err != paNoError) {
      result.reason = std::string("Pa_WriteStream: ") + Pa_GetErrorText(err);
      break;
    }
    buf += chunk * channels;
    frames_to_write -= chunk;
  }

  if (!impl_->stop_requested && result.reason.empty()) {
    // Wait for remaining data to finish playing (max playback duration + 2s).
    auto play_ms = std::chrono::milliseconds(
      static_cast<long long>(pcm.size()) / channels * 1000 / sample_rate);
    auto timeout = start + play_ms + std::chrono::milliseconds(2000);
    while (Pa_IsStreamActive(stream) && std::chrono::steady_clock::now() < timeout) {
      Pa_Sleep(50);
    }
  }

  Pa_StopStream(stream);
  Pa_CloseStream(stream);

  if (result.reason.empty()) {
    result.success = true;
  }
  auto end = std::chrono::steady_clock::now();
  result.duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  return result;
}

AudioDevice::PlaybackResult PortAudioDevice::play_preset(
  const std::string & preset_name)
{
  PlaybackResult result;
  if (!impl_->pa_initialized) {
    result.reason = "PortAudio not initialized";
    return result;
  }

  if (preset_name == "wake_ack") {
    // Load WAV from configured path and play as PCM.
    const std::string path = impl_->config.preset_wake_ack_path.empty()
      ? std::string{"/home/bianbu/.cache/assets/audio/wozai.wav"}
      : impl_->config.preset_wake_ack_path;
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
      result.reason = "cannot open preset: " + path;
      return result;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> raw(size);
    if (!file.read(raw.data(), size)) {
      result.reason = "cannot read preset: " + path;
      return result;
    }

    // Parse WAV header (44 bytes), extract PCM data
    if (size < 44) {
      result.reason = "preset WAV too short";
      return result;
    }

    // RIFF header: sample_rate at offset 24 (uint32), channels at 22 (uint16),
    // bits_per_sample at 34 (uint16), data size at 40 (uint32)
    uint32_t wav_sr, data_sz;
    uint16_t wav_ch, wav_bps;
    std::memcpy(&wav_sr, raw.data() + 24, sizeof(wav_sr));
    std::memcpy(&wav_ch, raw.data() + 22, sizeof(wav_ch));
    std::memcpy(&wav_bps, raw.data() + 34, sizeof(wav_bps));
    std::memcpy(&data_sz, raw.data() + 40, sizeof(data_sz));

    if (wav_bps != 16) {
      result.reason = "preset WAV not 16-bit";
      return result;
    }

    // Convert char* to int16_t*
    size_t sample_count = data_sz / 2;
    const int16_t* samples = reinterpret_cast<const int16_t*>(raw.data() + 44);

    // If sample rate mismatch, skip resampling and just play as-is
    // (wozai.wav should already be 16kHz mono)
    std::vector<int16_t> pcm(samples, samples + sample_count);

    std::cout << "[portaudio] Playing preset '" << preset_name
              << "' from " << path << " ("
              << wav_sr << "Hz, " << static_cast<int>(wav_ch) << "ch, "
              << (data_sz * 1000 / wav_sr / wav_ch / 2) << "ms)" << std::endl;

    return play_pcm(pcm, wav_sr, static_cast<uint8_t>(wav_ch), "s16le");
  }

  result.reason = "unknown preset: " + preset_name;
  return result;
}

void PortAudioDevice::stop()
{
  impl_->stop_requested = true;
  if (impl_->stream) {
    Pa_AbortStream(impl_->stream);
    Pa_CloseStream(impl_->stream);
    impl_->stream = nullptr;
  }
}

}  // namespace k1muse_voice_audio
