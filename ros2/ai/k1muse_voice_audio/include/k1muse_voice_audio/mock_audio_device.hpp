#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "k1muse_voice_audio/audio_device.hpp"

namespace k1muse_voice_audio
{

class MockAudioDevice : public AudioDevice
{
public:
  MockAudioDevice();
  explicit MockAudioDevice(std::chrono::milliseconds mock_delay);
  ~MockAudioDevice() override = default;

  const std::string & name() const override;
  void load() override;
  void unload() override;
  bool loaded() const override;

  PlaybackResult play_pcm(
    const std::vector<int16_t> & pcm, uint32_t sample_rate,
    uint8_t channels, const std::string & encoding) override;

  PlaybackResult play_preset(const std::string & preset_name) override;

  void stop() override;

  // Configure the duration for a named preset (for testing).
  void set_preset_duration(
    const std::string & preset_name, std::chrono::milliseconds duration);

private:
  bool interruptible_sleep(std::chrono::milliseconds duration);

  std::string name_{"mock_audio_device"};
  bool loaded_{false};
  std::atomic<bool> stop_requested_{false};
  std::chrono::milliseconds mock_delay_{0};
  bool use_mock_delay_{false};

  std::map<std::string, std::chrono::milliseconds> preset_durations_;

  mutable std::mutex sleep_mutex_;
  std::condition_variable sleep_cv_;
};

}  // namespace k1muse_voice_audio
