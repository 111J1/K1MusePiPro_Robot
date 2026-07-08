#include "k1muse_ai_runtime/audio/vad_segmenter.hpp"

#include <algorithm>

namespace k1muse_ai_runtime
{

VadSegmenter::VadSegmenter(const Config & config)
: config_(config)
, samples_per_chunk_(config.sample_rate * config.audio_chunk_ms / 1000)
, pre_roll_frames_(config.pre_roll_ms / config.audio_chunk_ms)
, post_pad_frames_((config.post_pad_ms + config.audio_chunk_ms - 1) / config.audio_chunk_ms)
{
}

VadSegmenter::SegmentResult VadSegmenter::process(
  const int16_t * pcm, size_t samples, float vad_probability)
{
  SegmentResult result;
  result.sample_rate = config_.sample_rate;

  const bool speech = (vad_probability >= config_.vad_threshold);

  switch (state_) {
    case State::Armed:
      if (speech) {
        // Transition to PreSpeech
        state_ = State::PreSpeech;
        speech_frame_count_ = 1;
        copy_pre_roll_to_active();
        append_to_active(pcm, samples);
      }
      break;

    case State::PreSpeech:
      if (speech) {
        pre_speech_silence_ = 0;
        ++speech_frame_count_;
        append_to_active(pcm, samples);
        if (speech_frame_count_ * config_.audio_chunk_ms >= config_.min_speech_ms) {
          state_ = State::InSpeech;
        }
      } else {
        // Hysteresis: tolerate 2 frames of borderline VAD before rejecting.
        // Real VAD models may flicker near threshold during speech onset.
        ++pre_speech_silence_;
        if (pre_speech_silence_ >= 2) {
          do_reset();
        }
      }
      break;

    case State::InSpeech:
      if (speech) {
        ++speech_frame_count_;
        append_to_active(pcm, samples);
        if (speech_frame_count_ * config_.audio_chunk_ms >= config_.max_utterance_ms) {
          // Forced endpoint — max utterance exceeded
          append_silence_to_active(post_pad_frames_);
          result.ready = true;
          result.pcm = std::move(active_segment_buffer_);
          do_reset();
        }
      } else {
        // Silence detected — transition to EndingSilence
        state_ = State::EndingSilence;
        silence_frame_count_ = 1;
        // Append this chunk so we can trim it later if segment completes,
        // or keep it if speech resumes.
        append_to_active(pcm, samples);
      }
      break;

    case State::EndingSilence:
      // Always append during EndingSilence (keeps buffer complete if speech resumes)
      append_to_active(pcm, samples);

      if (speech) {
        // Speech resumes — back to InSpeech
        speech_frame_count_ += silence_frame_count_ + 1;
        silence_frame_count_ = 0;
        state_ = State::InSpeech;
      } else {
        ++silence_frame_count_;
        if (silence_frame_count_ * config_.audio_chunk_ms >= config_.end_silence_ms) {
          // End silence threshold reached — trim trailing silence, add post-pad
          const size_t trim_samples =
            static_cast<size_t>(silence_frame_count_) * samples_per_chunk_;
          if (trim_samples <= active_segment_buffer_.size()) {
            active_segment_buffer_.resize(active_segment_buffer_.size() - trim_samples);
          }
          append_silence_to_active(post_pad_frames_);
          result.ready = true;
          result.pcm = std::move(active_segment_buffer_);
          do_reset();
        }
      }
      break;
  }

  // Update pre-roll ring after state transitions so Armed→PreSpeech
  // copies only previous frames (not the current one).
  push_to_pre_roll(pcm, samples);

  result.state = state_;
  return result;
}

void VadSegmenter::reset()
{
  do_reset();
  pre_roll_ring_.clear();
}

void VadSegmenter::feed_pre_roll(const int16_t * pcm, size_t samples)
{
  push_to_pre_roll(pcm, samples);
}

VadSegmenter::State VadSegmenter::state() const
{
  return state_;
}

void VadSegmenter::push_to_pre_roll(const int16_t * pcm, size_t samples)
{
  pre_roll_ring_.emplace_back(pcm, pcm + samples);
  while (pre_roll_ring_.size() > pre_roll_frames_) {
    pre_roll_ring_.pop_front();
  }
}

void VadSegmenter::copy_pre_roll_to_active()
{
  for (const auto & frame : pre_roll_ring_) {
    active_segment_buffer_.insert(
      active_segment_buffer_.end(), frame.begin(), frame.end());
  }
}

void VadSegmenter::append_to_active(const int16_t * pcm, size_t samples)
{
  active_segment_buffer_.insert(active_segment_buffer_.end(), pcm, pcm + samples);
}

void VadSegmenter::append_silence_to_active(uint32_t num_frames)
{
  const size_t silence_samples = static_cast<size_t>(num_frames) * samples_per_chunk_;
  active_segment_buffer_.insert(active_segment_buffer_.end(), silence_samples, 0);
}

void VadSegmenter::do_reset()
{
  state_ = State::Armed;
  active_segment_buffer_.clear();
  speech_frame_count_ = 0;
  silence_frame_count_ = 0;
}

}  // namespace k1muse_ai_runtime
