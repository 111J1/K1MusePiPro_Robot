#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/audio/audio_frame_validator.hpp"
#include "k1muse_ai_runtime/audio/audio_frame_queue.hpp"

using k1muse_ai_runtime::AudioFrameValidator;
using k1muse_ai_runtime::AudioFrameQueue;
using k1muse_ai_runtime::ValidationResult;

namespace
{

// Default expected format: 20ms, 16kHz, mono, S16LE = 320 samples
constexpr uint32_t kSampleRate = 16000;
constexpr uint8_t kChannels = 1;
constexpr uint16_t kFrameMs = 20;
const std::string kEncoding = "s16le";
constexpr std::size_t kExpectedSamples = 320;  // 16000 * 20 / 1000

std::vector<int16_t> make_pcm(std::size_t count)
{
  return std::vector<int16_t>(count, 0);
}

std::vector<int16_t> valid_pcm()
{
  return make_pcm(kExpectedSamples);
}

}  // namespace

// --- AudioFrameValidator tests ---

TEST(AudioFrameValidator, ValidFrame)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto result = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 1, now, now);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.reason.empty());
  EXPECT_FALSE(result.seq_gap);
  EXPECT_FALSE(result.format_change);
}

TEST(AudioFrameValidator, WrongSampleRate)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto result = validator.validate(
    8000, kChannels, kEncoding, kFrameMs,
    make_pcm(160), 1, now, now);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("sample_rate"), std::string::npos);
}

TEST(AudioFrameValidator, WrongChannels)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto result = validator.validate(
    kSampleRate, 2, kEncoding, kFrameMs,
    make_pcm(640), 1, now, now);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("channels"), std::string::npos);
}

TEST(AudioFrameValidator, WrongEncoding)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto result = validator.validate(
    kSampleRate, kChannels, "s16be", kFrameMs,
    valid_pcm(), 1, now, now);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("encoding"), std::string::npos);
}

TEST(AudioFrameValidator, WrongFrameMs)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto result = validator.validate(
    kSampleRate, kChannels, kEncoding, 10,
    make_pcm(160), 1, now, now);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("frame_ms"), std::string::npos);
}

TEST(AudioFrameValidator, WrongSampleCount)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto result = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    make_pcm(100), 1, now, now);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("pcm size"), std::string::npos);
}

TEST(AudioFrameValidator, SeqContinuity)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto r1 = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 1, now, now);
  EXPECT_TRUE(r1.valid);
  EXPECT_FALSE(r1.seq_gap);

  auto r2 = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 2, now, now);
  EXPECT_TRUE(r2.valid);
  EXPECT_FALSE(r2.seq_gap);

  auto r3 = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 3, now, now);
  EXPECT_TRUE(r3.valid);
  EXPECT_FALSE(r3.seq_gap);
}

TEST(AudioFrameValidator, SeqGap)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto r1 = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 1, now, now);
  EXPECT_TRUE(r1.valid);
  EXPECT_FALSE(r1.seq_gap);

  auto r2 = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 3, now, now);
  EXPECT_TRUE(r2.valid);
  EXPECT_TRUE(r2.seq_gap);
}

TEST(AudioFrameValidator, SeqReset)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto r1 = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 5, now, now);
  EXPECT_TRUE(r1.valid);
  EXPECT_FALSE(r1.seq_gap);

  validator.reset();

  // After reset, seq 1 should not be a gap (reset cleared state)
  auto r2 = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    valid_pcm(), 1, now, now);
  EXPECT_TRUE(r2.valid);
  EXPECT_FALSE(r2.seq_gap);
}

TEST(AudioFrameValidator, EmptyPcm)
{
  AudioFrameValidator validator;
  const auto now = std::chrono::steady_clock::now();
  auto result = validator.validate(
    kSampleRate, kChannels, kEncoding, kFrameMs,
    std::vector<int16_t>(), 1, now, now);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("pcm size"), std::string::npos);
}

// --- AudioFrameQueue tests ---

TEST(AudioFrameQueue, PushAndPop)
{
  AudioFrameQueue<int> queue(4);
  EXPECT_TRUE(queue.push(1));
  EXPECT_TRUE(queue.push(2));
  EXPECT_EQ(queue.size(), 2U);
  auto val = queue.try_pop();
  ASSERT_TRUE(val.has_value());
  EXPECT_EQ(val.value(), 1);
  val = queue.try_pop();
  ASSERT_TRUE(val.has_value());
  EXPECT_EQ(val.value(), 2);
  EXPECT_TRUE(queue.try_pop() == std::nullopt);
}

TEST(AudioFrameQueue, FullRejects)
{
  AudioFrameQueue<int> queue(2);
  EXPECT_TRUE(queue.push(1));
  EXPECT_TRUE(queue.push(2));
  EXPECT_FALSE(queue.push(3));  // rejected: full
  EXPECT_EQ(queue.size(), 2U);
}

TEST(AudioFrameQueue, PopWithTimeout)
{
  AudioFrameQueue<int> queue(4);
  auto start = std::chrono::steady_clock::now();
  auto val = queue.pop_with_timeout(std::chrono::milliseconds(50));
  auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_FALSE(val.has_value());
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 20);

  // Now push and verify it wakes up
  std::thread producer([&queue]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    queue.push(42);
  });
  val = queue.pop_with_timeout(std::chrono::milliseconds(200));
  ASSERT_TRUE(val.has_value());
  EXPECT_EQ(val.value(), 42);
  producer.join();
}

TEST(AudioFrameQueue, Clear)
{
  AudioFrameQueue<int> queue(4);
  queue.push(1);
  queue.push(2);
  EXPECT_EQ(queue.size(), 2U);
  queue.clear();
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_TRUE(queue.try_pop() == std::nullopt);
}
