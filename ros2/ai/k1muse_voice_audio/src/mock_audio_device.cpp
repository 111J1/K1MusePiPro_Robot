#include "k1muse_voice_audio/mock_audio_device.hpp"

#include <algorithm>
#include <thread>

namespace k1muse_voice_audio
{

MockAudioDevice::MockAudioDevice()
{
  // Default preset durations
  preset_durations_["wake_ack"] = std::chrono::milliseconds(500);
  preset_durations_["error_ack"] = std::chrono::milliseconds(300);
  preset_durations_["shutdown_ack"] = std::chrono::milliseconds(400);
}

MockAudioDevice::MockAudioDevice(std::chrono::milliseconds mock_delay)
: MockAudioDevice()
{
  mock_delay_ = mock_delay;
  use_mock_delay_ = true;
}

const std::string & MockAudioDevice::name() const
{
  return name_;
}

void MockAudioDevice::load()
{
  loaded_ = true;
}

void MockAudioDevice::unload()
{
  loaded_ = false;
}

bool MockAudioDevice::loaded() const
{
  return loaded_;
}

AudioDevice::PlaybackResult MockAudioDevice::play_pcm(
  const std::vector<int16_t> & pcm, uint32_t sample_rate,
  uint8_t /*channels*/, const std::string & /*encoding*/)
{
  if (!loaded_) {
    return {false, std::chrono::milliseconds{0}, "device not loaded"};
  }

  stop_requested_ = false;

  std::chrono::milliseconds duration;
  if (use_mock_delay_) {
    duration = mock_delay_;
  } else {
    // Calculate duration from PCM size and sample rate
    if (sample_rate == 0) {
      return {false, std::chrono::milliseconds{0}, "sample_rate is zero"};
    }
    auto samples = static_cast<double>(pcm.size());
    auto seconds = samples / static_cast<double>(sample_rate);
    duration = std::chrono::milliseconds(
      static_cast<int64_t>(seconds * 1000.0));
  }

  bool interrupted = interruptible_sleep(duration);
  if (interrupted) {
    return {false, std::chrono::milliseconds{0}, "playback stopped"};
  }

  return {true, duration, {}};
}

AudioDevice::PlaybackResult MockAudioDevice::play_preset(const std::string & preset_name)
{
  if (!loaded_) {
    return {false, std::chrono::milliseconds{0}, "device not loaded"};
  }

  stop_requested_ = false;

  auto it = preset_durations_.find(preset_name);
  std::chrono::milliseconds duration;
  if (it != preset_durations_.end()) {
    duration = it->second;
  } else {
    duration = std::chrono::milliseconds(200);  // Default for unknown presets
  }

  bool interrupted = interruptible_sleep(duration);
  if (interrupted) {
    return {false, std::chrono::milliseconds{0}, "playback stopped"};
  }

  return {true, duration, {}};
}

void MockAudioDevice::stop()
{
  stop_requested_ = true;
  sleep_cv_.notify_all();
}

void MockAudioDevice::set_preset_duration(
  const std::string & preset_name, std::chrono::milliseconds duration)
{
  preset_durations_[preset_name] = duration;
}

bool MockAudioDevice::interruptible_sleep(std::chrono::milliseconds duration)
{
  std::unique_lock<std::mutex> lock(sleep_mutex_);
  return sleep_cv_.wait_for(lock, duration, [this]() {
    return stop_requested_.load();
  });
}

}  // namespace k1muse_voice_audio
