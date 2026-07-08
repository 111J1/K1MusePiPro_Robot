#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace k1muse_ai_runtime
{

class VadSegmenter
{
public:
  enum class State { Armed, PreSpeech, InSpeech, EndingSilence, SegmentReady };

  struct Config
  {
    uint32_t sample_rate = 16000;
    uint32_t audio_chunk_ms = 20;
    float vad_threshold = 0.5f;
    uint32_t min_speech_ms = 250;
    uint32_t end_silence_ms = 500;
    uint32_t pre_roll_ms = 300;
    uint32_t post_pad_ms = 150;
    uint32_t max_utterance_ms = 10000;
  };

  struct SegmentResult
  {
    bool ready = false;
    std::vector<int16_t> pcm;
    uint32_t sample_rate = 0;
    State state = State::Armed;
  };

  explicit VadSegmenter(const Config & config);

  /// Process one audio chunk with its VAD probability.
  /// Returns SegmentResult with ready=true when a complete segment is available.
  SegmentResult process(const int16_t * pcm, size_t samples, float vad_probability);

  /// Reset state to Armed, clear all buffers including pre-roll.
  void reset();

  /// Reset state to Armed, keep pre-roll ring intact.
  void do_reset();

  /// Feed audio into pre-roll ring only — no VAD processing.
  /// Keeps the 300ms buffer warm when VAD is disabled.
  void feed_pre_roll(const int16_t * pcm, size_t samples);

  State state() const;

private:
  void push_to_pre_roll(const int16_t * pcm, size_t samples);
  void copy_pre_roll_to_active();
  void append_to_active(const int16_t * pcm, size_t samples);
  void append_silence_to_active(uint32_t num_frames);

  Config config_;
  uint32_t samples_per_chunk_;
  uint32_t pre_roll_frames_;
  uint32_t post_pad_frames_;

  State state_{State::Armed};
  uint32_t speech_frame_count_{0};
  uint32_t silence_frame_count_{0};
  uint32_t pre_speech_silence_{0};  // hysteresis counter for PreSpeech→Armed

  std::deque<std::vector<int16_t>> pre_roll_ring_;
  std::vector<int16_t> active_segment_buffer_;
};

}  // namespace k1muse_ai_runtime
