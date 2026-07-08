#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_mock_devices/audio_scenario.hpp"

using k1muse_mock_devices::AudioScenario;

namespace
{

constexpr uint32_t kSampleRate = 16000;
constexpr uint8_t kChannels = 1;
constexpr uint16_t kFrameMs = 20;
const std::string kEncoding = "s16le";
constexpr std::size_t kExpectedSamples = 320;  // 16000 * 20 / 1000
const std::string kTraceId = "test-trace";

AudioScenario::Config default_config()
{
  AudioScenario::Config cfg;
  cfg.sample_rate = kSampleRate;
  cfg.frame_ms = kFrameMs;
  cfg.channels = kChannels;
  cfg.frames_per_scenario = 50;
  cfg.wake_marker_value = 0x7FFF;
  cfg.speech_level = 5000;
  cfg.seq_start = 1;
  return cfg;
}

}  // namespace

// ---------------------------------------------------------------------------
// WakeMarkerFrame: First frame has trigger sample value
// ---------------------------------------------------------------------------

TEST(AudioScenario, WakeMarkerFrame)
{
  AudioScenario scenario(default_config());
  auto frames = scenario.generate(AudioScenario::Type::WakeMarker, kTraceId);

  ASSERT_FALSE(frames.empty());

  // First frame must contain the trigger sample
  const auto & first = frames[0];
  ASSERT_FALSE(first.pcm.empty());
  EXPECT_EQ(first.pcm[0], 0x7FFF);

  // Rest of first frame should be silence
  for (std::size_t i = 1; i < first.pcm.size(); ++i) {
    EXPECT_EQ(first.pcm[i], 0) << "sample " << i << " in first frame";
  }
}

// ---------------------------------------------------------------------------
// SpeechFrames: Speech frames have non-zero PCM
// ---------------------------------------------------------------------------

TEST(AudioScenario, SpeechFrames)
{
  AudioScenario scenario(default_config());
  auto frames = scenario.generate(AudioScenario::Type::Speech, kTraceId);

  ASSERT_FALSE(frames.empty());

  for (const auto & f : frames) {
    ASSERT_EQ(f.pcm.size(), kExpectedSamples);
    // At least some samples should be non-zero
    bool has_nonzero = false;
    for (auto s : f.pcm) {
      if (s != 0) {
        has_nonzero = true;
        break;
      }
    }
    EXPECT_TRUE(has_nonzero)
      << "Speech frame seq=" << f.seq << " is entirely zero";
  }
}

// ---------------------------------------------------------------------------
// SilenceFrames: Silence frames have zero PCM
// ---------------------------------------------------------------------------

TEST(AudioScenario, SilenceFrames)
{
  AudioScenario scenario(default_config());
  auto frames = scenario.generate(AudioScenario::Type::Silence, kTraceId);

  ASSERT_FALSE(frames.empty());

  for (const auto & f : frames) {
    ASSERT_EQ(f.pcm.size(), kExpectedSamples);
    for (std::size_t i = 0; i < f.pcm.size(); ++i) {
      EXPECT_EQ(f.pcm[i], 0)
        << "seq=" << f.seq << " sample " << i;
    }
  }
}

// ---------------------------------------------------------------------------
// SeqGapScenario: Seq numbers skip correctly
// ---------------------------------------------------------------------------

TEST(AudioScenario, SeqGapScenario)
{
  AudioScenario scenario(default_config());
  auto frames = scenario.generate(AudioScenario::Type::SeqGap, kTraceId);

  ASSERT_EQ(frames.size(), 20U);

  // First 10: seq 1..10
  for (uint32_t i = 0; i < 10; ++i) {
    EXPECT_EQ(frames[i].seq, 1U + i) << "first block frame " << i;
  }
  // Next 10: seq 20..29
  for (uint32_t i = 0; i < 10; ++i) {
    EXPECT_EQ(frames[10 + i].seq, 20U + i) << "second block frame " << i;
  }

  // Verify the gap exists
  EXPECT_EQ(frames[9].seq, 10U);
  EXPECT_EQ(frames[10].seq, 20U);
}

// ---------------------------------------------------------------------------
// FrameFormat: All frames have correct sample_rate, channels, encoding, frame_ms
// ---------------------------------------------------------------------------

TEST(AudioScenario, FrameFormat)
{
  AudioScenario scenario(default_config());

  auto check_frames = [&](AudioScenario::Type type, const char * label) {
    auto frames = scenario.generate(type, kTraceId);
    ASSERT_FALSE(frames.empty()) << label;
    for (const auto & f : frames) {
      EXPECT_EQ(f.sample_rate, kSampleRate) << label << " seq=" << f.seq;
      EXPECT_EQ(f.channels, kChannels) << label << " seq=" << f.seq;
      EXPECT_EQ(f.encoding, kEncoding) << label << " seq=" << f.seq;
      EXPECT_EQ(f.frame_ms, kFrameMs) << label << " seq=" << f.seq;
      EXPECT_EQ(f.pcm.size(), kExpectedSamples) << label << " seq=" << f.seq;
      EXPECT_EQ(f.trace_id, kTraceId) << label << " seq=" << f.seq;
    }
  };

  check_frames(AudioScenario::Type::WakeMarker, "WakeMarker");
  check_frames(AudioScenario::Type::Speech, "Speech");
  check_frames(AudioScenario::Type::Silence, "Silence");
  check_frames(AudioScenario::Type::SeqGap, "SeqGap");
  check_frames(AudioScenario::Type::NoSpeech, "NoSpeech");
}

// ---------------------------------------------------------------------------
// NoSpeechFrames: NoSpeech produces silence (zero PCM)
// ---------------------------------------------------------------------------

TEST(AudioScenario, NoSpeechFrames)
{
  AudioScenario scenario(default_config());
  auto frames = scenario.generate(AudioScenario::Type::NoSpeech, kTraceId);

  ASSERT_FALSE(frames.empty());
  for (const auto & f : frames) {
    for (auto s : f.pcm) {
      EXPECT_EQ(s, 0);
    }
  }
}
