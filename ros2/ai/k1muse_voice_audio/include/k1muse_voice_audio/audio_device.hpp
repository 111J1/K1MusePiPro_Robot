#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_voice_audio
{

class AudioDevice
{
public:
  virtual ~AudioDevice() = default;
  virtual const std::string & name() const = 0;
  virtual void load() = 0;
  virtual void unload() = 0;
  virtual bool loaded() const = 0;

  struct PlaybackResult
  {
    bool success = false;
    std::chrono::milliseconds duration{0};
    std::string reason;
  };

  // Play PCM audio. Returns after playback completes (blocking for mock).
  virtual PlaybackResult play_pcm(
    const std::vector<int16_t> & pcm, uint32_t sample_rate,
    uint8_t channels, const std::string & encoding) = 0;

  // Play a named preset (e.g., "wake_ack"). Returns after playback completes.
  virtual PlaybackResult play_preset(const std::string & preset_name) = 0;

  virtual void stop() = 0;
};

}  // namespace k1muse_voice_audio
