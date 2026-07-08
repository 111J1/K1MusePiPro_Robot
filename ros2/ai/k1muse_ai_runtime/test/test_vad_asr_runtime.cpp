#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/backends/mock_asr_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_vad_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_wakeword_backend.hpp"
#include "k1muse_ai_runtime/control_snapshot.hpp"
#include "k1muse_ai_runtime/models/vad_asr_runtime.hpp"
#include "k1muse_ai_runtime/models/wakeword_runtime.hpp"

using k1muse_ai_runtime::ControlSnapshot;
using k1muse_ai_runtime::CancellationToken;
using k1muse_ai_runtime::MockAsrBackend;
using k1muse_ai_runtime::MockVadBackend;
using k1muse_ai_runtime::MockWakewordBackend;
using k1muse_ai_runtime::VadAsrRuntime;
using k1muse_ai_runtime::VadSegmenter;
using k1muse_ai_runtime::WakewordRuntime;

namespace
{

constexpr std::size_t kSamplesPerChunk = 320;  // 16000 * 20 / 1000

// High-amplitude samples produce RMS > 0.5 when normalised by MockVadBackend.
constexpr int16_t kSpeechAmplitude = 20000;

ControlSnapshot make_control(
  uint64_t epoch, bool wakeword_enabled, bool vad_asr_enabled)
{
  ControlSnapshot c;
  c.epoch = epoch;
  c.trace_id = "test_trace";
  c.wakeword_enabled = wakeword_enabled;
  c.vad_asr_enabled = vad_asr_enabled;
  return c;
}

std::vector<int16_t> make_chunk(int16_t value)
{
  return std::vector<int16_t>(kSamplesPerChunk, value);
}

// --- Wakeword helpers ---

struct WakewordResult
{
  int call_count{0};
  std::string trace_id;
  uint64_t epoch{0};
  float confidence{0.0f};
  std::string keyword;
};

std::unique_ptr<WakewordRuntime> make_wakeword_runtime(WakewordResult & result)
{
  auto backend = std::make_unique<MockWakewordBackend>();
  auto callback = [&result](
    const std::string & trace_id, uint64_t epoch,
    float confidence, const std::string & keyword) {
      ++result.call_count;
      result.trace_id = trace_id;
      result.epoch = epoch;
      result.confidence = confidence;
      result.keyword = keyword;
    };
  return std::make_unique<WakewordRuntime>(std::move(backend), callback);
}

// --- VadAsr helpers ---

struct EventRecord
{
  std::string trace_id;
  std::string utterance_id;
  uint64_t epoch{0};
  VadAsrRuntime::ListenEvent event;
  std::string reason;
};

struct ResultRecord
{
  std::string trace_id;
  std::string utterance_id;
  uint64_t epoch{0};
  bool success{false};
  std::string text;
  float confidence{0.0f};
  std::string language;
  std::string reason;
};

struct VadAsrResult
{
  std::vector<EventRecord> events;
  std::vector<ResultRecord> results;
};

std::unique_ptr<VadAsrRuntime> make_vad_asr_runtime(VadAsrResult & result)
{
  auto vad = std::make_unique<MockVadBackend>();
  auto asr = std::make_unique<MockAsrBackend>();
  VadSegmenter::Config config;

  auto event_cb = [&result](
    const std::string & trace_id, const std::string & utterance_id,
    uint64_t epoch, VadAsrRuntime::ListenEvent event,
    const std::string & reason) {
      result.events.push_back({trace_id, utterance_id, epoch, event, reason});
    };
  auto result_cb = [&result](
    const std::string & trace_id, const std::string & utterance_id,
    uint64_t epoch, bool success, const std::string & text,
    float confidence, const std::string & language,
    const std::string & reason) {
      result.results.push_back(
        {trace_id, utterance_id, epoch, success, text,
          confidence, language, reason});
    };
  return std::make_unique<VadAsrRuntime>(
    std::move(vad), std::move(asr), config,
    "cpu",
    std::move(event_cb), std::move(result_cb));
}

void feed_speech_n(VadAsrRuntime & runtime, const ControlSnapshot & control,
  uint64_t & seq, int count)
{
  auto chunk = make_chunk(kSpeechAmplitude);
  for (int i = 0; i < count; ++i) {
    runtime.process_audio(
      chunk.data(), chunk.size(), control.trace_id, control.epoch, control,
      seq++);
  }
}

void feed_silence_n(VadAsrRuntime & runtime, const ControlSnapshot & control,
  uint64_t & seq, int count)
{
  auto chunk = make_chunk(0);
  for (int i = 0; i < count; ++i) {
    runtime.process_audio(
      chunk.data(), chunk.size(), control.trace_id, control.epoch, control,
      seq++);
  }
}

void feed_vad_only_speech_n(VadAsrRuntime & runtime, const ControlSnapshot & control,
  uint64_t & seq, int count)
{
  auto chunk = make_chunk(kSpeechAmplitude);
  for (int i = 0; i < count; ++i) {
    runtime.process_vad_only(
      chunk.data(), chunk.size(), control.trace_id, control.epoch, control,
      seq++);
  }
}

}  // namespace

// --- Test 1: WakewordDetection ---
TEST(VadAsrRuntimeTest, WakewordDetection)
{
  WakewordResult result;
  auto runtime = make_wakeword_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Chunk containing the trigger sample (0x7FFF).
  auto chunk = make_chunk(100);
  chunk[kSamplesPerChunk / 2] = 0x7FFF;

  ControlSnapshot control = make_control(1, true, false);
  runtime->process_audio(
    chunk.data(), chunk.size(), "trace_wake", 1, control);

  EXPECT_EQ(result.call_count, 1);
  EXPECT_EQ(result.trace_id, "trace_wake");
  EXPECT_EQ(result.epoch, 1U);
  EXPECT_FLOAT_EQ(result.confidence, 1.0f);
  EXPECT_EQ(result.keyword, "hello");

  runtime->unload();
}

// --- Test 2: WakewordDisabled ---
TEST(VadAsrRuntimeTest, WakewordDisabled)
{
  WakewordResult result;
  auto runtime = make_wakeword_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  auto chunk = make_chunk(100);
  chunk[kSamplesPerChunk / 2] = 0x7FFF;

  // wakeword_enabled = false.
  ControlSnapshot control = make_control(1, false, false);
  runtime->process_audio(
    chunk.data(), chunk.size(), "trace_disabled", 1, control);

  EXPECT_EQ(result.call_count, 0);

  runtime->unload();
}

// --- Test 3: VadSpeechDetection ---
TEST(VadAsrRuntimeTest, VadSpeechDetection)
{
  VadAsrResult result;
  auto runtime = make_vad_asr_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  ControlSnapshot control = make_control(1, false, true);
  uint64_t seq = 1;

  // Feed enough speech frames for Armed -> PreSpeech.
  // MockVadBackend with amplitude 20000 returns RMS ~0.61 > threshold 0.5.
  // Segmenter transitions Armed -> PreSpeech on first speech frame.
  // min_speech_ms=250, chunk_ms=20 -> need 13 frames for InSpeech.
  // Feed 5 frames to stay in PreSpeech (short noise window).
  feed_speech_n(*runtime, control, seq, 5);

  // SPEECH_START should have fired on the transition Armed -> PreSpeech.
  ASSERT_EQ(result.events.size(), 1U);
  EXPECT_EQ(result.events[0].event, VadAsrRuntime::ListenEvent::SPEECH_START);
  EXPECT_EQ(result.events[0].epoch, 1U);
  EXPECT_FALSE(result.events[0].utterance_id.empty());
  // No ASR result yet (segment not complete).
  EXPECT_TRUE(result.results.empty());

  runtime->unload();
}

// Runtime workers call process_vad_only(), not process_audio().
// Consecutive enabled frames must not be treated as sequence gaps.
TEST(VadAsrRuntimeTest, VadOnlySpeechDetection)
{
  VadAsrResult result;
  auto runtime = make_vad_asr_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  ControlSnapshot control = make_control(1, false, true);
  uint64_t seq = 1;

  feed_vad_only_speech_n(*runtime, control, seq, 5);

  ASSERT_EQ(result.events.size(), 1U);
  EXPECT_EQ(result.events[0].event, VadAsrRuntime::ListenEvent::SPEECH_START);
  EXPECT_EQ(runtime->vad_state(), VadSegmenter::State::PreSpeech);
  EXPECT_TRUE(result.results.empty());

  runtime->unload();
}
// --- Test 4: VadSegmentReady ---
TEST(VadAsrRuntimeTest, VadSegmentReady)
{
  VadAsrResult result;
  auto runtime = make_vad_asr_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  ControlSnapshot control = make_control(1, false, true);
  uint64_t seq = 1;

  // 13 speech frames -> InSpeech.
  feed_speech_n(*runtime, control, seq, 13);
  // 25 silence frames -> SegmentReady.
  feed_silence_n(*runtime, control, seq, 25);

  // Expect SPEECH_START + SPEECH_END.
  ASSERT_GE(result.events.size(), 2U);
  EXPECT_EQ(result.events[0].event, VadAsrRuntime::ListenEvent::SPEECH_START);
  EXPECT_EQ(result.events[1].event, VadAsrRuntime::ListenEvent::SPEECH_END);
  // Both events share the same utterance_id.
  EXPECT_EQ(result.events[0].utterance_id, result.events[1].utterance_id);
  // ASR should have processed the segment.
  ASSERT_EQ(result.results.size(), 1U);
  EXPECT_TRUE(result.results[0].success);

  runtime->unload();
}

// --- Test 5: AsrResultPublished ---
TEST(VadAsrRuntimeTest, AsrResultPublished)
{
  VadAsrResult result;
  auto runtime = make_vad_asr_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  ControlSnapshot control = make_control(2, false, true);
  uint64_t seq = 1;

  // Complete utterance: 13 speech + 25 silence.
  feed_speech_n(*runtime, control, seq, 13);
  feed_silence_n(*runtime, control, seq, 25);

  ASSERT_EQ(result.results.size(), 1U);
  EXPECT_EQ(result.results[0].trace_id, "test_trace");
  EXPECT_EQ(result.results[0].epoch, 2U);
  EXPECT_TRUE(result.results[0].success);
  EXPECT_EQ(result.results[0].text, "mock transcription");
  EXPECT_FLOAT_EQ(result.results[0].confidence, 0.95f);
  EXPECT_EQ(result.results[0].language, "zh");
  EXPECT_TRUE(result.results[0].reason.empty());
  // VAD state should have reset to Armed after SegmentReady.
  EXPECT_EQ(runtime->vad_state(), VadSegmenter::State::Armed);

  runtime->unload();
}

// --- Test 6: EpochMismatchDropped ---
TEST(VadAsrRuntimeTest, EpochMismatchDropped)
{
  VadAsrResult result;
  auto runtime = make_vad_asr_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Start with epoch=3.
  ControlSnapshot control = make_control(3, false, true);
  uint64_t seq = 1;

  // Feed 13 speech frames at epoch 3.
  feed_speech_n(*runtime, control, seq, 13);

  // Change epoch to 4 (simulates supervisor update).
  control.epoch = 4;

  // Feed 25 silence frames to complete the segment.
  // The segment was started at epoch 3, but control is now epoch 4.
  feed_silence_n(*runtime, control, seq, 25);

  // Events (SPEECH_START, SPEECH_END) still fire since they don't gate on epoch.
  EXPECT_GE(result.events.size(), 2U);
  // ASR result should be dropped due to epoch mismatch.
  EXPECT_TRUE(result.results.empty());

  runtime->unload();
}

// --- Test 7: SeqGapResetsVad ---
TEST(VadAsrRuntimeTest, SeqGapResetsVad)
{
  VadAsrResult result;
  auto runtime = make_vad_asr_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  ControlSnapshot control = make_control(1, false, true);
  uint64_t seq = 1;

  // Feed 5 speech frames (seq 1..5), VAD is in PreSpeech.
  feed_speech_n(*runtime, control, seq, 5);
  EXPECT_EQ(runtime->vad_state(), VadSegmenter::State::PreSpeech);

  // Simulate a sequence gap: jump to seq 10.
  seq = 10;
  auto chunk = make_chunk(kSpeechAmplitude);
  runtime->process_audio(
    chunk.data(), chunk.size(), control.trace_id, control.epoch, control,
    seq);

  // VAD should have been reset to Armed due to the gap.
  EXPECT_EQ(runtime->vad_state(), VadSegmenter::State::Armed);

  runtime->unload();
}
