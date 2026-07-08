#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/backends/mock_tts_backend.hpp"
#include "k1muse_ai_runtime/inference_job.hpp"
#include "k1muse_ai_runtime/models/tts_runtime.hpp"

using k1muse_ai_runtime::CancellationToken;
using k1muse_ai_runtime::JobModule;
using k1muse_ai_runtime::MockTtsBackend;
using k1muse_ai_runtime::TTSRuntime;
using k1muse_ai_runtime::TtsBackend;

namespace
{

// TtsStatus state constants (mirroring TtsStatus.msg).
constexpr uint8_t STATE_PENDING = 0;
constexpr uint8_t STATE_DONE = 2;
constexpr uint8_t STATE_FAILED = 3;
constexpr uint8_t STATE_CANCELLED = 4;

// TtsTextRequest priority constants.
constexpr uint8_t PRIORITY_DEBUG = 0;
constexpr uint8_t PRIORITY_USER_REPLY = 1;
constexpr uint8_t PRIORITY_REMINDER = 2;
constexpr uint8_t PRIORITY_SYSTEM = 3;

struct StatusRecord
{
  std::string trace_id;
  std::string request_id;
  uint64_t epoch{0};
  std::string source;
  uint8_t state{0};
  std::string state_name;
  std::string reason;
};

struct PlayRecord
{
  std::string trace_id;
  std::string request_id;
  uint64_t epoch{0};
  std::string source;
  uint32_t sample_rate{0};
  uint8_t channels{0};
  std::string encoding;
  std::vector<int16_t> pcm;
};

struct CallbackLog
{
  std::vector<StatusRecord> statuses;
  std::vector<PlayRecord> plays;
};

std::unique_ptr<TTSRuntime> make_tts_runtime(CallbackLog & log)
{
  auto backend = std::make_unique<MockTtsBackend>();
  TTSRuntime::StatusCallback callbacks;
  callbacks.publish_status = [&log](
    const std::string & trace_id, const std::string & request_id,
    uint64_t epoch, const std::string & source,
    uint8_t state, const std::string & state_name,
    const std::string & reason) {
      log.statuses.push_back(
        {trace_id, request_id, epoch, source, state, state_name, reason});
    };
  callbacks.publish_play = [&log](
    const std::string & trace_id, const std::string & request_id,
    uint64_t epoch, const std::string & source,
    const TTSRuntime::PcmResult & pcm) {
      PlayRecord rec;
      rec.trace_id = trace_id;
      rec.request_id = request_id;
      rec.epoch = epoch;
      rec.source = source;
      rec.sample_rate = pcm.sample_rate;
      rec.channels = pcm.channels;
      rec.encoding = pcm.encoding;
      rec.pcm = pcm.pcm_s16le;
      log.plays.push_back(std::move(rec));
    };
  return std::make_unique<TTSRuntime>(
    std::move(backend), "cpu", std::move(callbacks));
}

auto epoch_true = [](uint64_t) { return true; };
auto module_enabled_true = [](JobModule) { return true; };
auto cancel_false = []() { return false; };

}  // namespace

// --- Test 1: SubmitAndProcess ---
// submit + process -> status DONE + play request published.
TEST(TTSRuntimeTest, SubmitAndProcess)
{
  CallbackLog log;
  auto runtime = make_tts_runtime(log);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Submit a request.
  bool accepted = runtime->submit_request(
    "trace_1", "req_1", 1, "asr", PRIORITY_USER_REPLY, "hello world", "default");
  EXPECT_TRUE(accepted);
  EXPECT_TRUE(runtime->has_pending());

  // Expect: QUEUED status published.
  ASSERT_EQ(log.statuses.size(), 1U);
  EXPECT_EQ(log.statuses[0].state, STATE_PENDING);
  EXPECT_EQ(log.statuses[0].state_name, "pending");
  EXPECT_EQ(log.statuses[0].trace_id, "trace_1");
  EXPECT_EQ(log.statuses[0].request_id, "req_1");

  // Process.
  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_pending(
    epoch_true, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 1, true);

  EXPECT_TRUE(processed);
  EXPECT_FALSE(runtime->has_pending());

  // Expect: DONE status + play request.
  ASSERT_EQ(log.statuses.size(), 2U);
  EXPECT_EQ(log.statuses[1].state, STATE_DONE);
  EXPECT_EQ(log.statuses[1].state_name, "done");

  ASSERT_EQ(log.plays.size(), 1U);
  EXPECT_EQ(log.plays[0].trace_id, "trace_1");
  EXPECT_EQ(log.plays[0].request_id, "req_1");
  EXPECT_EQ(log.plays[0].epoch, 1U);
  EXPECT_FALSE(log.plays[0].pcm.empty());

  runtime->unload();
}

// --- Test 2: HigherPriorityReplaces ---
// submit PRIORITY_USER_REPLY, submit PRIORITY_SYSTEM -> old replaced.
TEST(TTSRuntimeTest, HigherPriorityReplaces)
{
  CallbackLog log;
  auto runtime = make_tts_runtime(log);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Submit low priority.
  bool accepted1 = runtime->submit_request(
    "trace_lo", "req_lo", 1, "asr", PRIORITY_USER_REPLY, "low text", "default");
  EXPECT_TRUE(accepted1);

  // Submit high priority.
  bool accepted2 = runtime->submit_request(
    "trace_hi", "req_hi", 1, "system", PRIORITY_SYSTEM, "high text", "default");
  EXPECT_TRUE(accepted2);

  // Expect: pending(status_lo) + cancelled(status_lo) + pending(status_hi)
  ASSERT_EQ(log.statuses.size(), 3U);
  EXPECT_EQ(log.statuses[0].state, STATE_PENDING);  // first accepted
  EXPECT_EQ(log.statuses[0].request_id, "req_lo");
  EXPECT_EQ(log.statuses[1].state, STATE_CANCELLED);  // first cancelled
  EXPECT_EQ(log.statuses[1].request_id, "req_lo");
  EXPECT_EQ(log.statuses[2].state, STATE_PENDING);  // second accepted
  EXPECT_EQ(log.statuses[2].request_id, "req_hi");

  // Process should use the high priority request.
  auto now = std::chrono::steady_clock::now();
  runtime->process_pending(
    epoch_true, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 1, true);

  ASSERT_EQ(log.plays.size(), 1U);
  EXPECT_EQ(log.plays[0].trace_id, "trace_hi");
  EXPECT_EQ(log.plays[0].request_id, "req_hi");

  runtime->unload();
}

// --- Test 3: LowerPriorityRejected ---
// submit PRIORITY_SYSTEM, submit PRIORITY_DEBUG -> new rejected.
TEST(TTSRuntimeTest, LowerPriorityRejected)
{
  CallbackLog log;
  auto runtime = make_tts_runtime(log);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Submit high priority.
  bool accepted1 = runtime->submit_request(
    "trace_hi", "req_hi", 1, "system", PRIORITY_SYSTEM, "important", "default");
  EXPECT_TRUE(accepted1);

  // Submit low priority -- should be rejected.
  bool accepted2 = runtime->submit_request(
    "trace_lo", "req_lo", 1, "debug", PRIORITY_DEBUG, "debug text", "default");
  EXPECT_FALSE(accepted2);

  // Expect: pending(hi) + rejected(lo)
  ASSERT_EQ(log.statuses.size(), 2U);
  EXPECT_EQ(log.statuses[0].state, STATE_PENDING);
  EXPECT_EQ(log.statuses[0].request_id, "req_hi");
  EXPECT_EQ(log.statuses[1].state, STATE_FAILED);
  EXPECT_EQ(log.statuses[1].state_name, "rejected");
  EXPECT_EQ(log.statuses[1].request_id, "req_lo");

  // Pending should still be the high priority request.
  EXPECT_TRUE(runtime->has_pending());

  // Process should use the high priority request.
  auto now = std::chrono::steady_clock::now();
  runtime->process_pending(
    epoch_true, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 1, true);

  ASSERT_EQ(log.plays.size(), 1U);
  EXPECT_EQ(log.plays[0].trace_id, "trace_hi");

  runtime->unload();
}

// --- Test 4: DisabledCaches ---
// tts_enabled=false, submit -> QUEUED. Enable -> process -> DONE.
TEST(TTSRuntimeTest, DisabledCaches)
{
  CallbackLog log;
  auto runtime = make_tts_runtime(log);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Submit while TTS is conceptually enabled (submit always accepts).
  bool accepted = runtime->submit_request(
    "trace_1", "req_1", 1, "asr", PRIORITY_USER_REPLY, "hello", "default");
  EXPECT_TRUE(accepted);

  // Process with tts_enabled=false -> should return false without consuming.
  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_pending(
    epoch_true, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 1, false);
  EXPECT_FALSE(processed);
  EXPECT_TRUE(runtime->has_pending());

  // Process with tts_enabled=true -> should succeed.
  processed = runtime->process_pending(
    epoch_true, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 1, true);
  EXPECT_TRUE(processed);
  EXPECT_FALSE(runtime->has_pending());

  // Expect: pending + done + play.
  ASSERT_EQ(log.statuses.size(), 2U);
  EXPECT_EQ(log.statuses[0].state, STATE_PENDING);
  EXPECT_EQ(log.statuses[1].state, STATE_DONE);
  ASSERT_EQ(log.plays.size(), 1U);

  runtime->unload();
}

// --- Test 5: EmptyTextFails ---
// submit empty text -> FAILED status on process.
TEST(TTSRuntimeTest, EmptyTextFails)
{
  CallbackLog log;
  auto runtime = make_tts_runtime(log);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Submit request with empty text.
  bool accepted = runtime->submit_request(
    "trace_1", "req_1", 1, "asr", PRIORITY_USER_REPLY, "", "default");
  EXPECT_TRUE(accepted);

  // Process -> should fail with empty text.
  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_pending(
    epoch_true, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 1, true);
  EXPECT_TRUE(processed);
  EXPECT_FALSE(runtime->has_pending());

  // Expect: pending + failed.
  ASSERT_EQ(log.statuses.size(), 2U);
  EXPECT_EQ(log.statuses[0].state, STATE_PENDING);
  EXPECT_EQ(log.statuses[1].state, STATE_FAILED);
  EXPECT_EQ(log.statuses[1].reason, "empty text");

  // No play request.
  EXPECT_EQ(log.plays.size(), 0U);

  runtime->unload();
}

// --- Test 6: StaleEpochDropped ---
// epoch changes between submit and process -> no publish.
TEST(TTSRuntimeTest, StaleEpochDropped)
{
  CallbackLog log;
  auto runtime = make_tts_runtime(log);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Submit at epoch 1.
  bool accepted = runtime->submit_request(
    "trace_1", "req_1", 1, "asr", PRIORITY_USER_REPLY, "hello", "default");
  EXPECT_TRUE(accepted);

  // Process with epoch_is_current that returns false for epoch 1.
  auto epoch_not_current = [](uint64_t epoch) { return epoch != 1; };
  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_pending(
    epoch_not_current, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 2, true);

  // Request was consumed but stale -> silently dropped.
  EXPECT_TRUE(processed);
  EXPECT_FALSE(runtime->has_pending());

  // Expect: only the initial QUEUED status. No DONE/FAILED, no play.
  ASSERT_EQ(log.statuses.size(), 1U);
  EXPECT_EQ(log.statuses[0].state, STATE_PENDING);
  EXPECT_EQ(log.plays.size(), 0U);

  runtime->unload();
}

// --- Test 7: ClearPendingWorks ---
// submit, clear, has_pending=false.
TEST(TTSRuntimeTest, ClearPendingWorks)
{
  CallbackLog log;
  auto runtime = make_tts_runtime(log);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  bool accepted = runtime->submit_request(
    "trace_1", "req_1", 1, "asr", PRIORITY_USER_REPLY, "hello", "default");
  EXPECT_TRUE(accepted);
  EXPECT_TRUE(runtime->has_pending());

  runtime->clear_pending();
  EXPECT_FALSE(runtime->has_pending());

  // Process should return false (no pending).
  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_pending(
    epoch_true, module_enabled_true, cancel_false,
    now + std::chrono::seconds(1), 1, true);
  EXPECT_FALSE(processed);

  runtime->unload();
}
