#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

struct ValidationResult
{
  bool valid{false};
  std::string reason;
  bool seq_gap{false};
  bool format_change{false};
};

struct AudioFrameParams
{
  uint32_t sample_rate{16000};
  uint8_t channels{1};
  std::string encoding{"s16le"};
  uint16_t frame_ms{20};
};

class AudioFrameValidator
{
public:
  explicit AudioFrameValidator(
    uint32_t expected_sample_rate = 16000,
    uint8_t expected_channels = 1,
    const std::string & expected_encoding = "s16le",
    uint16_t expected_frame_ms = 20,
    std::chrono::milliseconds max_frame_age = std::chrono::milliseconds(500));

  ValidationResult validate(
    uint32_t sample_rate,
    uint8_t channels,
    const std::string & encoding,
    uint16_t frame_ms,
    const std::vector<int16_t> & pcm,
    uint32_t seq,
    std::chrono::steady_clock::time_point header_stamp,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

  void reset();

  AudioFrameParams expected_params() const;

private:
  uint32_t expected_sample_rate_;
  uint8_t expected_channels_;
  std::string expected_encoding_;
  uint16_t expected_frame_ms_;
  std::size_t expected_samples_;
  std::chrono::milliseconds max_frame_age_;

  bool has_last_seq_{false};
  uint32_t last_seq_{0};
  AudioFrameParams last_params_;
};

}  // namespace k1muse_ai_runtime
