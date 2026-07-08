#include "k1muse_ai_runtime/audio/audio_frame_validator.hpp"

namespace k1muse_ai_runtime
{

AudioFrameValidator::AudioFrameValidator(
  uint32_t expected_sample_rate,
  uint8_t expected_channels,
  const std::string & expected_encoding,
  uint16_t expected_frame_ms,
  std::chrono::milliseconds max_frame_age)
: expected_sample_rate_(expected_sample_rate)
, expected_channels_(expected_channels)
, expected_encoding_(expected_encoding)
, expected_frame_ms_(expected_frame_ms)
, expected_samples_(
    static_cast<std::size_t>(expected_sample_rate) * expected_frame_ms / 1000)
, max_frame_age_(max_frame_age)
{
}

ValidationResult AudioFrameValidator::validate(
  uint32_t sample_rate,
  uint8_t channels,
  const std::string & encoding,
  uint16_t frame_ms,
  const std::vector<int16_t> & pcm,
  uint32_t seq,
  std::chrono::steady_clock::time_point header_stamp,
  std::chrono::steady_clock::time_point now)
{
  ValidationResult result;

  // Check format fields
  const bool rate_ok = (sample_rate == expected_sample_rate_);
  const bool channels_ok = (channels == expected_channels_);
  const bool encoding_ok = (encoding == expected_encoding_);
  const bool frame_ms_ok = (frame_ms == expected_frame_ms_);
  const bool sample_count_ok = (pcm.size() == expected_samples_);

  // Detect format change from last frame
  if (has_last_seq_) {
    if (sample_rate != last_params_.sample_rate ||
      channels != last_params_.channels ||
      encoding != last_params_.encoding ||
      frame_ms != last_params_.frame_ms)
    {
      result.format_change = true;
    }
  }

  if (!rate_ok) {
    result.reason = "sample_rate mismatch: expected " +
      std::to_string(expected_sample_rate_) + ", got " +
      std::to_string(sample_rate);
  } else if (!channels_ok) {
    result.reason = "channels mismatch: expected " +
      std::to_string(expected_channels_) + ", got " +
      std::to_string(channels);
  } else if (!encoding_ok) {
    result.reason = "encoding mismatch: expected " +
      expected_encoding_ + ", got " + encoding;
  } else if (!frame_ms_ok) {
    result.reason = "frame_ms mismatch: expected " +
      std::to_string(expected_frame_ms_) + ", got " +
      std::to_string(frame_ms);
  } else if (!sample_count_ok) {
    result.reason = "pcm size mismatch: expected " +
      std::to_string(expected_samples_) + ", got " +
      std::to_string(pcm.size());
  }

  if (!rate_ok || !channels_ok || !encoding_ok || !frame_ms_ok || !sample_count_ok) {
    // Still update seq tracking even on format errors
    last_seq_ = seq;
    has_last_seq_ = true;
    last_params_ = {sample_rate, channels, encoding, frame_ms};
    return result;
  }

  // Seq continuity check
  if (has_last_seq_) {
    const uint32_t expected_seq = last_seq_ + 1;
    if (seq != expected_seq) {
      result.seq_gap = true;
    }
  }

  // Frame age check
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - header_stamp);
  if (age > max_frame_age_) {
    result.reason = "frame too old: " + std::to_string(age.count()) + "ms";
    // Still update tracking
    last_seq_ = seq;
    has_last_seq_ = true;
    last_params_ = {sample_rate, channels, encoding, frame_ms};
    return result;
  }

  // Valid
  result.valid = true;
  last_seq_ = seq;
  has_last_seq_ = true;
  last_params_ = {sample_rate, channels, encoding, frame_ms};
  return result;
}

void AudioFrameValidator::reset()
{
  has_last_seq_ = false;
  last_seq_ = 0;
  last_params_ = {};
}

AudioFrameParams AudioFrameValidator::expected_params() const
{
  return AudioFrameParams{
    expected_sample_rate_,
    expected_channels_,
    expected_encoding_,
    expected_frame_ms_};
}

}  // namespace k1muse_ai_runtime
