#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/audio/vad_segmenter.hpp"

using k1muse_ai_runtime::VadSegmenter;

namespace
{

constexpr uint32_t kSampleRate = 16000;
constexpr uint32_t kChunkMs = 20;
constexpr std::size_t kSamplesPerChunk = 320;  // 16000 * 20 / 1000

// Default config for most tests
VadSegmenter::Config default_config()
{
  return VadSegmenter::Config{};
}

// Create a chunk filled with a specific value (for tracing pre-roll origin)
std::vector<int16_t> make_chunk(int16_t value)
{
  return std::vector<int16_t>(kSamplesPerChunk, value);
}

// Feed one chunk and return the result
VadSegmenter::SegmentResult feed(VadSegmenter & seg, int16_t value, float prob)
{
  auto chunk = make_chunk(value);
  return seg.process(chunk.data(), chunk.size(), prob);
}

// Feed N chunks with the same value and probability (no result checked)
void feed_n(VadSegmenter & seg, int16_t value, float prob, uint32_t n)
{
  for (uint32_t i = 0; i < n; ++i) {
    feed(seg, value, prob);
  }
}

}  // namespace

// --- Test 1: ArmedToPreSpeech ---
TEST(VadSegmenter, ArmedToPreSpeech)
{
  VadSegmenter seg(default_config());
  EXPECT_EQ(seg.state(), VadSegmenter::State::Armed);

  auto result = feed(seg, 100, 1.0f);
  EXPECT_EQ(seg.state(), VadSegmenter::State::PreSpeech);
  EXPECT_FALSE(result.ready);
}

// --- Test 2: PreSpeechToInSpeech ---
TEST(VadSegmenter, PreSpeechToInSpeech)
{
  VadSegmenter seg(default_config());

  // Feed min_speech_ms / chunk_ms = 250/20 = 12.5 → need 13 frames
  for (uint32_t i = 0; i < 12; ++i) {
    auto result = feed(seg, 100, 1.0f);
    EXPECT_EQ(seg.state(), VadSegmenter::State::PreSpeech)
      << "Expected PreSpeech at frame " << (i + 1);
  }

  // 13th frame should transition to InSpeech
  auto result = feed(seg, 100, 1.0f);
  EXPECT_EQ(seg.state(), VadSegmenter::State::InSpeech);
  EXPECT_FALSE(result.ready);
}

// --- Test 3: ShortNoiseRejected ---
TEST(VadSegmenter, ShortNoiseRejected)
{
  VadSegmenter seg(default_config());

  // Feed 5 speech frames (less than min_speech_ms threshold of 13)
  feed_n(seg, 100, 1.0f, 5);
  EXPECT_EQ(seg.state(), VadSegmenter::State::PreSpeech);

  // Silence — should reject and return to Armed
  auto result = feed(seg, 100, 0.0f);
  EXPECT_EQ(seg.state(), VadSegmenter::State::Armed);
  EXPECT_FALSE(result.ready);
}

// --- Test 4: InSpeechToEndingSilence ---
TEST(VadSegmenter, InSpeechToEndingSilence)
{
  VadSegmenter seg(default_config());

  // Get to InSpeech
  feed_n(seg, 100, 1.0f, 13);
  EXPECT_EQ(seg.state(), VadSegmenter::State::InSpeech);

  // Silence — should transition to EndingSilence
  auto result = feed(seg, 100, 0.0f);
  EXPECT_EQ(seg.state(), VadSegmenter::State::EndingSilence);
  EXPECT_FALSE(result.ready);
}

// --- Test 5: EndingSilenceResumes ---
TEST(VadSegmenter, EndingSilenceResumes)
{
  VadSegmenter seg(default_config());

  // Get to InSpeech
  feed_n(seg, 100, 1.0f, 13);
  EXPECT_EQ(seg.state(), VadSegmenter::State::InSpeech);

  // Some silence (not enough for SegmentReady)
  feed_n(seg, 100, 0.0f, 5);
  EXPECT_EQ(seg.state(), VadSegmenter::State::EndingSilence);

  // Speech resumes — back to InSpeech
  auto result = feed(seg, 100, 1.0f);
  EXPECT_EQ(seg.state(), VadSegmenter::State::InSpeech);
  EXPECT_FALSE(result.ready);
}

// --- Test 6: SegmentReady ---
TEST(VadSegmenter, SegmentReady)
{
  VadSegmenter seg(default_config());

  // Get to InSpeech (13 frames)
  feed_n(seg, 100, 1.0f, 13);
  EXPECT_EQ(seg.state(), VadSegmenter::State::InSpeech);

  // Feed end_silence_ms / chunk_ms = 500/20 = 25 silence frames
  VadSegmenter::SegmentResult result;
  for (uint32_t i = 0; i < 24; ++i) {
    result = feed(seg, 100, 0.0f);
    EXPECT_FALSE(result.ready) << "Should not be ready at silence frame " << (i + 1);
  }

  // 25th silence frame triggers SegmentReady
  result = feed(seg, 100, 0.0f);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.pcm.empty());
  EXPECT_EQ(result.sample_rate, kSampleRate);

  // After SegmentReady, state should be Armed
  EXPECT_EQ(seg.state(), VadSegmenter::State::Armed);

  // Verify PCM size: 13 speech frames + 8 post-pad frames = 21 frames
  // (pre-roll was empty since no frames were fed before speech)
  const std::size_t expected_frames = 13 + 8;  // speech + post_pad
  EXPECT_EQ(result.pcm.size(), expected_frames * kSamplesPerChunk);
}

// --- Test 7: MaxUtteranceForced ---
TEST(VadSegmenter, MaxUtteranceForced)
{
  VadSegmenter seg(default_config());

  // max_utterance_ms = 10000, chunk_ms = 20 → 500 frames total
  VadSegmenter::SegmentResult result;
  for (uint32_t i = 0; i < 499; ++i) {
    result = feed(seg, 100, 1.0f);
    EXPECT_FALSE(result.ready) << "Should not be ready at frame " << (i + 1);
  }

  // 500th frame triggers forced SegmentReady
  result = feed(seg, 100, 1.0f);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.pcm.empty());

  // After SegmentReady, state should be Armed
  EXPECT_EQ(seg.state(), VadSegmenter::State::Armed);

  // Verify PCM size: 500 speech frames + 8 post-pad frames = 508 frames
  const std::size_t expected_frames = 500 + 8;
  EXPECT_EQ(result.pcm.size(), expected_frames * kSamplesPerChunk);
}

// --- Test 8: PreRollIncluded ---
TEST(VadSegmenter, PreRollIncluded)
{
  VadSegmenter seg(default_config());

  // Feed 20 background frames (low probability, value=50)
  feed_n(seg, 50, 0.0f, 20);

  // Feed 13 speech frames (high probability, value=200)
  feed_n(seg, 200, 1.0f, 13);

  // Feed 25 silence frames to trigger SegmentReady
  VadSegmenter::SegmentResult result;
  for (uint32_t i = 0; i < 25; ++i) {
    result = feed(seg, 200, 0.0f);
  }
  EXPECT_TRUE(result.ready);

  // The segment should contain:
  // - Pre-roll: last 15 background frames (value=50) before speech
  // - Speech: 13 frames (value=200)
  // - Post-pad: 8 frames of silence (value=0)
  const std::size_t pre_roll_samples = 15 * kSamplesPerChunk;
  const std::size_t speech_samples = 13 * kSamplesPerChunk;
  const std::size_t post_pad_samples = 8 * kSamplesPerChunk;
  const std::size_t expected_total = pre_roll_samples + speech_samples + post_pad_samples;

  EXPECT_EQ(result.pcm.size(), expected_total);

  // Verify pre-roll region has background value
  for (std::size_t i = 0; i < pre_roll_samples; ++i) {
    EXPECT_EQ(result.pcm[i], 50)
      << "Pre-roll sample " << i << " should be 50";
  }

  // Verify speech region has speech value
  for (std::size_t i = pre_roll_samples; i < pre_roll_samples + speech_samples; ++i) {
    EXPECT_EQ(result.pcm[i], 200)
      << "Speech sample " << i << " should be 200";
  }

  // Verify post-pad region is silence
  for (std::size_t i = pre_roll_samples + speech_samples; i < expected_total; ++i) {
    EXPECT_EQ(result.pcm[i], 0)
      << "Post-pad sample " << i << " should be 0";
  }
}

// --- Test 9: ResetToArmed ---
TEST(VadSegmenter, ResetToArmed)
{
  VadSegmenter seg(default_config());

  // Get to InSpeech
  feed_n(seg, 100, 1.0f, 15);
  EXPECT_EQ(seg.state(), VadSegmenter::State::InSpeech);

  // Reset
  seg.reset();
  EXPECT_EQ(seg.state(), VadSegmenter::State::Armed);

  // After reset, feeding speech should start fresh from Armed → PreSpeech
  auto result = feed(seg, 100, 1.0f);
  EXPECT_EQ(seg.state(), VadSegmenter::State::PreSpeech);
}

// --- Test 10: MultipleSegments ---
TEST(VadSegmenter, MultipleSegments)
{
  VadSegmenter seg(default_config());
  uint32_t segment_count = 0;

  auto run_segment = [&]() {
    // Get to InSpeech (13 frames)
    feed_n(seg, 100, 1.0f, 13);

    // Silence to trigger SegmentReady (25 frames)
    for (uint32_t i = 0; i < 25; ++i) {
      auto result = feed(seg, 100, 0.0f);
      if (result.ready) {
        ++segment_count;
        EXPECT_FALSE(result.pcm.empty());
      }
    }
  };

  // First segment
  run_segment();
  EXPECT_EQ(segment_count, 1u);
  EXPECT_EQ(seg.state(), VadSegmenter::State::Armed);

  // Second segment
  run_segment();
  EXPECT_EQ(segment_count, 2u);
  EXPECT_EQ(seg.state(), VadSegmenter::State::Armed);
}
