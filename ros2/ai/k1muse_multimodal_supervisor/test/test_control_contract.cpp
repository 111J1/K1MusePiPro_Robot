#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_ai_runtime_msgs/msg/ai_runtime_control.hpp"
#include "k1muse_ai_runtime_msgs/msg/ai_runtime_state.hpp"
#include "k1muse_common/msg/node_ready.hpp"
#include "k1muse_multimodal_supervisor/multimodal_supervisor_node.hpp"
#include "k1muse_voice_msgs/msg/audio_play_request.hpp"
#include "k1muse_voice_msgs/msg/audio_playback_state.hpp"
#include "k1muse_voice_msgs/msg/intent_status.hpp"
#include "k1muse_voice_msgs/msg/listen_result.hpp"
#include "k1muse_voice_msgs/msg/tts_status.hpp"
#include "k1muse_voice_msgs/msg/wakeword_event.hpp"

namespace k1muse_multimodal_supervisor {
namespace {

using AiRuntimeState  = k1muse_ai_runtime_msgs::msg::AiRuntimeState;
using AiRuntimeControl = k1muse_ai_runtime_msgs::msg::AiRuntimeControl;
using WakewordEvent     = k1muse_voice_msgs::msg::WakewordEvent;
using ListenResult      = k1muse_voice_msgs::msg::ListenResult;
using IntentStatus      = k1muse_voice_msgs::msg::IntentStatus;
using TtsStatus         = k1muse_voice_msgs::msg::TtsStatus;
using AudioPlayRequest  = k1muse_voice_msgs::msg::AudioPlayRequest;
using AudioPlaybackState = k1muse_voice_msgs::msg::AudioPlaybackState;
using NodeReady         = k1muse_common::msg::NodeReady;

// ---------------------------------------------------------------------------
// Generic topic collector
// ---------------------------------------------------------------------------

template <typename T>
class MessageCollector {
 public:
  explicit MessageCollector(
      rclcpp::Node::SharedPtr node, const std::string& topic,
      const rclcpp::QoS& qos = rclcpp::QoS(10).reliable().durability_volatile()) {
    sub_ = node->create_subscription<T>(
        topic, qos,
        [this](typename T::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          msgs_.push_back(*msg);
          cv_.notify_all();
        });
  }

  bool wait_for(size_t count, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_.wait_for(lock, timeout, [this, count]() {
      return msgs_.size() >= count;
    });
  }

  const std::vector<T>& messages() const { return msgs_; }
  size_t size() const { return msgs_.size(); }

 private:
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  std::vector<T> msgs_;
  std::mutex mtx_;
  std::condition_variable cv_;
};

// ---------------------------------------------------------------------------
// Global test environment
// ---------------------------------------------------------------------------

class IntegrationTestEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

::testing::Environment* const kEnv =
    ::testing::AddGlobalTestEnvironment(new IntegrationTestEnvironment);

// ---------------------------------------------------------------------------
// Test fixture — same pattern as test_voice_flow_integration.cpp
// ---------------------------------------------------------------------------

class ControlContractTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("no_speech_timeout_sec", 1.0),
        rclcpp::Parameter("wake_ack_preset", std::string{"wake_ack"}),
        rclcpp::Parameter("target_cache_ttl_ms", 500),
    });
    node_ = std::make_shared<MultimodalSupervisorNode>(options);

    // Publishers — inject mock events on the topics the supervisor subscribes to.
    runtime_state_pub_ = node_->create_publisher<AiRuntimeState>(
        "/ai_runtime/state", rclcpp::QoS(1).reliable().transient_local());
    wakeword_pub_ = node_->create_publisher<WakewordEvent>(
        "/ai_runtime/wakeword/event", rclcpp::QoS(5).reliable().durability_volatile());
    listen_result_pub_ = node_->create_publisher<ListenResult>(
        "/voice/listen/result", rclcpp::QoS(10).reliable().durability_volatile());
    intent_status_pub_ = node_->create_publisher<IntentStatus>(
        "/voice/intent/status", rclcpp::QoS(5).reliable().durability_volatile());
    tts_status_pub_ = node_->create_publisher<TtsStatus>(
        "/voice/tts/status", rclcpp::QoS(1).reliable().transient_local());
    playback_state_pub_ = node_->create_publisher<AudioPlaybackState>(
        "/voice/audio/playback_state", rclcpp::QoS(1).reliable().transient_local());
    audio_io_pub_ = node_->create_publisher<NodeReady>(
        "/audio_io/state", rclcpp::QoS(1).reliable().transient_local());
    intent_state_pub_ = node_->create_publisher<NodeReady>(
        "/intent/state", rclcpp::QoS(1).reliable().transient_local());

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    spin_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });
  }

  void TearDown() override {
    executor_->cancel();
    if (spin_future_.valid()) {
      spin_future_.wait();
    }
    executor_->remove_node(node_);
    node_.reset();
    executor_.reset();
  }

  // ── Helpers ─────────────────────────────────────────────────

  void bring_to_idle() {
    MessageCollector<AiRuntimeControl> ctrl_col(
        node_, "/ai_runtime/control",
        rclcpp::QoS(10).reliable().transient_local());

    NodeReady audio_msg;
    audio_msg.header.stamp = node_->now();
    audio_msg.node_name = "audio_io";
    audio_msg.ready = true;
    audio_io_pub_->publish(audio_msg);

    NodeReady intent_msg;
    intent_msg.header.stamp = node_->now();
    intent_msg.node_name = "intent_node";
    intent_msg.ready = true;
    intent_state_pub_->publish(intent_msg);

    AiRuntimeState rt_msg;
    rt_msg.header.stamp = node_->now();
    rt_msg.runtime_ready = true;
    rt_msg.epoch = 0;
    runtime_state_pub_->publish(rt_msg);

    ASSERT_TRUE(ctrl_col.wait_for(2, std::chrono::milliseconds(3000)));
    ASSERT_EQ(node_->current_state(), State::IDLE);
  }

  AudioPlayRequest trigger_wakeword(
      MessageCollector<AudioPlayRequest>& play_col, uint64_t epoch = 1) {
    WakewordEvent msg;
    msg.header.stamp = node_->now();
    msg.trace_id = "test_trace";
    msg.epoch = epoch;
    msg.event = WakewordEvent::EVENT_DETECTED;
    msg.keyword = "hello";
    msg.confidence = 0.95f;
    wakeword_pub_->publish(msg);

    EXPECT_TRUE(play_col.wait_for(1, std::chrono::milliseconds(3000)));
    return play_col.messages().back();
  }

  void complete_wake_ack(const AudioPlayRequest& play_req) {
    AudioPlaybackState pb_msg;
    pb_msg.header.stamp = node_->now();
    pb_msg.request_id = play_req.request_id;
    pb_msg.epoch = play_req.epoch;
    pb_msg.source = "test";
    pb_msg.state = AudioPlaybackState::STATE_DONE;
    pb_msg.state_name = "DONE";
    playback_state_pub_->publish(pb_msg);
  }

  void send_listen_result(const std::string& text, const std::string& trace_id,
                          uint64_t epoch, const std::string& utterance_id = "u1") {
    ListenResult msg;
    msg.header.stamp = node_->now();
    msg.trace_id = trace_id;
    msg.utterance_id = utterance_id;
    msg.epoch = epoch;
    msg.success = true;
    msg.text = text;
    msg.confidence = 0.95f;
    msg.language = "zh";
    listen_result_pub_->publish(msg);
  }

  void send_intent_finished(const std::string& trace_id,
                            const std::string& request_id, uint64_t epoch,
                            const std::string& utterance_id = "u1",
                            bool has_tts = true) {
    IntentStatus msg;
    msg.header.stamp = node_->now();
    msg.trace_id = trace_id;
    msg.request_id = request_id;
    msg.utterance_id = utterance_id;
    msg.epoch = epoch;
    msg.state = IntentStatus::STATE_FINISHED;
    msg.state_name = "FINISHED";
    msg.has_tts = has_tts;
    intent_status_pub_->publish(msg);
  }

  void send_tts_done(const std::string& trace_id,
                     const std::string& request_id, uint64_t epoch) {
    TtsStatus msg;
    msg.header.stamp = node_->now();
    msg.trace_id = trace_id;
    msg.request_id = request_id;
    msg.epoch = epoch;
    msg.source = "tts";
    msg.state = TtsStatus::STATE_DONE;
    msg.state_name = "DONE";
    tts_status_pub_->publish(msg);
  }

  void send_tts_playback_done(const std::string& trace_id,
                              const std::string& request_id, uint64_t epoch) {
    AudioPlaybackState msg;
    msg.header.stamp = node_->now();
    msg.trace_id = trace_id;
    msg.request_id = request_id;
    msg.epoch = epoch;
    msg.source = "audio_io";
    msg.state = AudioPlaybackState::STATE_DONE;
    msg.state_name = "DONE";
    playback_state_pub_->publish(msg);
  }

  /// Assert that the last AiRuntimeControl matches expected flags.
  void verify_control(const MessageCollector<AiRuntimeControl>& col,
                      const std::string& expected_state_name,
                      bool wakeword, bool vision, bool vad_asr, bool tts) {
    ASSERT_GT(col.size(), 0u) << "No AiRuntimeControl messages collected";
    const auto& msg = col.messages().back();
    EXPECT_EQ(msg.interaction_state_name, expected_state_name);
    EXPECT_EQ(msg.wakeword_enabled, wakeword)
        << "wakeword mismatch in " << expected_state_name;
    EXPECT_EQ(msg.vision_enabled, vision)
        << "vision mismatch in " << expected_state_name;
    EXPECT_EQ(msg.vad_asr_enabled, vad_asr)
        << "vad_asr mismatch in " << expected_state_name;
    EXPECT_EQ(msg.tts_enabled, tts)
        << "tts mismatch in " << expected_state_name;
    EXPECT_FALSE(msg.trace_id.empty()) << "trace_id should not be empty";
  }

  std::shared_ptr<MultimodalSupervisorNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::future<void> spin_future_;

  rclcpp::Publisher<AiRuntimeState>::SharedPtr runtime_state_pub_;
  rclcpp::Publisher<WakewordEvent>::SharedPtr wakeword_pub_;
  rclcpp::Publisher<ListenResult>::SharedPtr listen_result_pub_;
  rclcpp::Publisher<IntentStatus>::SharedPtr intent_status_pub_;
  rclcpp::Publisher<TtsStatus>::SharedPtr tts_status_pub_;
  rclcpp::Publisher<AudioPlaybackState>::SharedPtr playback_state_pub_;
  rclcpp::Publisher<NodeReady>::SharedPtr audio_io_pub_;
  rclcpp::Publisher<NodeReady>::SharedPtr intent_state_pub_;
};

// ===========================================================================
// Test 1 — FullVoiceFlowControlFlags
// ===========================================================================

TEST_F(ControlContractTest, FullVoiceFlowControlFlags) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  // BOOT → IDLE
  bring_to_idle();
  ASSERT_EQ(node_->current_state(), State::IDLE);
  verify_control(ctrl_col, "IDLE", true, true, false, false);

  // IDLE → WAKE_ACK
  auto play_req = trigger_wakeword(play_col, 100);
  ASSERT_TRUE(ctrl_col.wait_for(3, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::WAKE_ACK);
  verify_control(ctrl_col, "WAKE_ACK", false, false, false, false);

  // WAKE_ACK → LISTENING
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);
  verify_control(ctrl_col, "LISTENING", false, false, true, false);

  // LISTENING → INTENT_PROCESSING
  send_listen_result("前进", play_req.trace_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::INTENT_PROCESSING);
  verify_control(ctrl_col, "INTENT_PROCESSING", false, false, false, false);

  // INTENT_PROCESSING → TTS_RUNNING
  std::string intent_req_id = "intent-req-001";
  send_intent_finished(play_req.trace_id, intent_req_id, play_req.epoch, "u1",
                       /*has_tts=*/true);
  ASSERT_TRUE(ctrl_col.wait_for(6, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);
  verify_control(ctrl_col, "TTS_RUNNING", false, false, false, true);

  // TTS_RUNNING → IDLE (dual-completion)
  send_tts_done(play_req.trace_id, intent_req_id, play_req.epoch);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);  // still waiting

  send_tts_playback_done(play_req.trace_id, intent_req_id, play_req.epoch);
  ASSERT_TRUE(ctrl_col.wait_for(8, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::IDLE);
  verify_control(ctrl_col, "IDLE", true, true, false, false);
}

// ===========================================================================
// Test 2 — NoSpeechTimeout
// ===========================================================================

TEST_F(ControlContractTest, NoSpeechTimeoutGoesIdle) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();

  auto play_req = trigger_wakeword(play_col, 200);
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);

  // Wait for no-speech timeout (1.0s) → back to IDLE.
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(5000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);
  verify_control(ctrl_col, "IDLE", true, true, false, false);
}

// ===========================================================================
// Test 3 — StaleEpochListenResultRejected
// ===========================================================================

TEST_F(ControlContractTest, StaleEpochListenResultRejected) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();
  auto play_req = trigger_wakeword(play_col, 300);
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);

  // Send stale-epoch ListenResult — should be rejected.
  send_listen_result("前进", play_req.trace_id, /*epoch=*/0, "u1");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(node_->current_state(), State::LISTENING);
  EXPECT_EQ(ctrl_col.size(), 4u);  // no new control message

  // Send correct-epoch ListenResult → accepted.
  send_listen_result("前进", play_req.trace_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::INTENT_PROCESSING);
}

// ===========================================================================
// Test 4 — IntentNoTtsSkipsRunning
// ===========================================================================

TEST_F(ControlContractTest, IntentNoTtsSkipsRunning) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();
  auto play_req = trigger_wakeword(play_col, 400);
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);

  send_listen_result("几点", play_req.trace_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::INTENT_PROCESSING);

  // has_tts=false → skip TTS_RUNNING, go directly to IDLE.
  send_intent_finished(play_req.trace_id, "intent-req-002", play_req.epoch,
                       "u1", /*has_tts=*/false);
  ASSERT_TRUE(ctrl_col.wait_for(6, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);

  // Verify none of the collected controls are TTS_RUNNING.
  for (const auto& c : ctrl_col.messages()) {
    EXPECT_NE(c.interaction_state_name, "TTS_RUNNING");
  }
}

// ===========================================================================
// Test 5 — TtsDualCompletionBothOrders
// ===========================================================================

TEST_F(ControlContractTest, TtsDualCompletionPlaybackFirst) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();
  auto play_req = trigger_wakeword(play_col, 500);
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);

  send_listen_result("前进", play_req.trace_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::INTENT_PROCESSING);

  std::string intent_req_id = "intent-req-003";
  send_intent_finished(play_req.trace_id, intent_req_id, play_req.epoch,
                       "u1", /*has_tts=*/true);
  ASSERT_TRUE(ctrl_col.wait_for(6, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);

  // Send PLAYBACK_DONE FIRST (reverse order from Test 1).
  send_tts_playback_done(play_req.trace_id, intent_req_id, play_req.epoch);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);  // still waiting for TTS_DONE

  // Now send TTS_DONE → both present → transition to IDLE.
  send_tts_done(play_req.trace_id, intent_req_id, play_req.epoch);
  ASSERT_TRUE(ctrl_col.wait_for(8, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);
  verify_control(ctrl_col, "IDLE", true, true, false, false);
}

}  // namespace
}  // namespace k1muse_multimodal_supervisor
