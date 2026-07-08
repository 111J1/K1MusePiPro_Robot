#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_ai_runtime_msgs/msg/ai_runtime_control.hpp"
#include "k1muse_ai_runtime_msgs/msg/ai_runtime_state.hpp"
#include "k1muse_common/msg/node_ready.hpp"
#include "k1muse_manager_msgs/msg/interaction_state.hpp"
#include "k1muse_multimodal_supervisor/multimodal_supervisor_node.hpp"
#include "k1muse_vision_msgs/msg/detection2_d_frame.hpp"
#include "k1muse_vision_msgs/msg/target_request.hpp"
#include "k1muse_vision_msgs/msg/target_response.hpp"
#include "k1muse_voice_msgs/msg/audio_play_request.hpp"
#include "k1muse_voice_msgs/msg/audio_playback_state.hpp"
#include "k1muse_voice_msgs/msg/intent_status.hpp"
#include "k1muse_voice_msgs/msg/listen_result.hpp"
#include "k1muse_voice_msgs/msg/tts_status.hpp"
#include "k1muse_voice_msgs/msg/wakeword_event.hpp"

namespace k1muse_multimodal_supervisor {
namespace {

using AiRuntimeState = k1muse_ai_runtime_msgs::msg::AiRuntimeState;
using AiRuntimeControl = k1muse_ai_runtime_msgs::msg::AiRuntimeControl;
using WakewordEvent = k1muse_voice_msgs::msg::WakewordEvent;
using ListenResult = k1muse_voice_msgs::msg::ListenResult;
using IntentStatus = k1muse_voice_msgs::msg::IntentStatus;
using TtsStatus = k1muse_voice_msgs::msg::TtsStatus;
using AudioPlayRequest = k1muse_voice_msgs::msg::AudioPlayRequest;
using AudioPlaybackState = k1muse_voice_msgs::msg::AudioPlaybackState;
using Detection2DFrame = k1muse_vision_msgs::msg::Detection2DFrame;
using TargetRequest = k1muse_vision_msgs::msg::TargetRequest;
using TargetResponse = k1muse_vision_msgs::msg::TargetResponse;
using NodeReady = k1muse_common::msg::NodeReady;
using InteractionState = k1muse_manager_msgs::msg::InteractionState;

// ---------------------------------------------------------------------------
// Generic message collector
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
// Test environment
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
// Test fixture
// ---------------------------------------------------------------------------

class VoiceFlowIntegrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("no_speech_timeout_sec", 1.5),
        rclcpp::Parameter("tts_completion_timeout_sec", 0.5),
        rclcpp::Parameter("wake_ack_preset", std::string{"wake_ack"}),
        rclcpp::Parameter("target_cache_ttl_ms", 500),
    });
    node_ = std::make_shared<MultimodalSupervisorNode>(options);

    // Helper publishers.
    runtime_state_pub_ = node_->create_publisher<AiRuntimeState>(
        "/ai_runtime/state", rclcpp::QoS(1).reliable().transient_local());
    wakeword_pub_ = node_->create_publisher<WakewordEvent>(
        "/voice/wakeword/event", rclcpp::QoS(5).reliable().durability_volatile());
    listen_result_pub_ = node_->create_publisher<ListenResult>(
        "/voice/listen/result", rclcpp::QoS(10).reliable().durability_volatile());
    intent_status_pub_ = node_->create_publisher<IntentStatus>(
        "/voice/intent/status", rclcpp::QoS(5).reliable().durability_volatile());
    tts_status_pub_ = node_->create_publisher<TtsStatus>(
        "/voice/tts/status", rclcpp::QoS(1).reliable().transient_local());
    playback_state_pub_ = node_->create_publisher<AudioPlaybackState>(
        "/voice/audio/playback_state", rclcpp::QoS(1).reliable().transient_local());
    detection_pub_ = node_->create_publisher<Detection2DFrame>(
        "/vision/detection_2d", rclcpp::QoS(5).reliable().durability_volatile());
    target_request_pub_ = node_->create_publisher<TargetRequest>(
        "/target/request", rclcpp::QoS(5).reliable().durability_volatile());
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

  // Bring the node to IDLE state.
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

  // Trigger wakeword and return the published AudioPlayRequest.
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

  // Complete wake-ack playback -> LISTENING.
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

  // Send ListenResult.
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

  // Send IntentStatus FINISHED.
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

  // Send TTS_DONE.
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

  // Send AudioPlaybackState DONE for TTS.
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

  std::shared_ptr<MultimodalSupervisorNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::future<void> spin_future_;

  rclcpp::Publisher<AiRuntimeState>::SharedPtr runtime_state_pub_;
  rclcpp::Publisher<WakewordEvent>::SharedPtr wakeword_pub_;
  rclcpp::Publisher<ListenResult>::SharedPtr listen_result_pub_;
  rclcpp::Publisher<IntentStatus>::SharedPtr intent_status_pub_;
  rclcpp::Publisher<TtsStatus>::SharedPtr tts_status_pub_;
  rclcpp::Publisher<AudioPlaybackState>::SharedPtr playback_state_pub_;
  rclcpp::Publisher<Detection2DFrame>::SharedPtr detection_pub_;
  rclcpp::Publisher<TargetRequest>::SharedPtr target_request_pub_;
  rclcpp::Publisher<NodeReady>::SharedPtr audio_io_pub_;
  rclcpp::Publisher<NodeReady>::SharedPtr intent_state_pub_;
};

// ---------------------------------------------------------------------------
// Test 1: FullVoiceFlow
// BOOT -> IDLE -> WAKE_ACK -> LISTENING -> INTENT_PROCESSING -> TTS_RUNNING -> IDLE
// ---------------------------------------------------------------------------

TEST_F(VoiceFlowIntegrationTest, FullVoiceFlow) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  // BOOT -> IDLE
  bring_to_idle();
  ASSERT_EQ(node_->current_state(), State::IDLE);

  // IDLE -> WAKE_ACK
  auto play_req = trigger_wakeword(play_col, 100);
  ASSERT_EQ(node_->current_state(), State::WAKE_ACK);

  // WAKE_ACK -> LISTENING
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);

  // LISTENING -> INTENT_PROCESSING
  send_listen_result("go forward", play_req.trace_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::INTENT_PROCESSING);

  // INTENT_PROCESSING -> TTS_RUNNING
  // IntentStatus uses a new request_id generated by the intent node.
  std::string intent_req_id = "intent-req-001";
  send_intent_finished(play_req.trace_id, intent_req_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(6, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);

  // TTS_RUNNING -> IDLE (dual-completion: TTS_DONE then TTS_PLAYBACK_DONE)
  send_tts_done(play_req.trace_id, intent_req_id, play_req.epoch);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);  // Still waiting.

  send_tts_playback_done(play_req.trace_id, intent_req_id, play_req.epoch);
  ASSERT_TRUE(ctrl_col.wait_for(8, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::IDLE);
}

TEST_F(VoiceFlowIntegrationTest, TtsDoneWithoutPlaybackTimesOutToIdle) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();
  auto play_req = trigger_wakeword(play_col, 600);
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);

  send_listen_result("go forward", play_req.trace_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(3000)));

  const std::string intent_req_id = "intent-req-timeout-tts";
  send_intent_finished(play_req.trace_id, intent_req_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(6, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);

  send_tts_done(play_req.trace_id, intent_req_id, play_req.epoch);
  ASSERT_TRUE(ctrl_col.wait_for(7, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);
}

TEST_F(VoiceFlowIntegrationTest, PlaybackDoneWithoutTtsDoneTimesOutToIdle) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();
  auto play_req = trigger_wakeword(play_col, 700);
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::LISTENING);

  send_listen_result("go forward", play_req.trace_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(3000)));

  const std::string intent_req_id = "intent-req-timeout-playback";
  send_intent_finished(play_req.trace_id, intent_req_id, play_req.epoch, "u1");
  ASSERT_TRUE(ctrl_col.wait_for(6, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::TTS_RUNNING);

  send_tts_playback_done(play_req.trace_id, intent_req_id, play_req.epoch);
  ASSERT_TRUE(ctrl_col.wait_for(7, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// Test 2: NoSpeechTimeout
// LISTENING -> timeout -> IDLE
// ---------------------------------------------------------------------------

TEST_F(VoiceFlowIntegrationTest, NoSpeechTimeout) {
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

  // Wait for no-speech timeout (configured to 1.5s).
  // Should transition back to IDLE.
  ASSERT_TRUE(ctrl_col.wait_for(5, std::chrono::milliseconds(5000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);
}

// ---------------------------------------------------------------------------
// Test 3: StalePlaybackRejected
// AudioPlaybackState with wrong request_id is ignored.
// ---------------------------------------------------------------------------

TEST_F(VoiceFlowIntegrationTest, StalePlaybackRejected) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();

  auto play_req = trigger_wakeword(play_col, 300);
  ASSERT_EQ(node_->current_state(), State::WAKE_ACK);

  // Publish playback DONE with WRONG request_id.
  AudioPlaybackState stale_msg;
  stale_msg.header.stamp = node_->now();
  stale_msg.request_id = "wrong_request_id";
  stale_msg.epoch = play_req.epoch;
  stale_msg.source = "test";
  stale_msg.state = AudioPlaybackState::STATE_DONE;
  stale_msg.state_name = "DONE";
  playback_state_pub_->publish(stale_msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Should still be in WAKE_ACK.
  EXPECT_EQ(node_->current_state(), State::WAKE_ACK);

  // Now send correct playback -> should transition to LISTENING.
  complete_wake_ack(play_req);
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::LISTENING);
}

// ---------------------------------------------------------------------------
// Test 4: TargetingFlow
// IDLE -> TARGETING -> IDLE
// ---------------------------------------------------------------------------

TEST_F(VoiceFlowIntegrationTest, TargetingFlow) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<TargetResponse> resp_col(
      node_, "/target/response",
      rclcpp::QoS(5).reliable().durability_volatile());

  bring_to_idle();
  ASSERT_EQ(node_->current_state(), State::IDLE);

  // First, feed a detection frame into the cache.
  Detection2DFrame det_msg;
  det_msg.header.stamp = node_->now();
  det_msg.image_width = 640;
  det_msg.image_height = 480;
  k1muse_vision_msgs::msg::Detection2D det;
  det.detection_id = "det-1";
  det.class_name = "cup";
  det.score = 0.95f;
  det.x = 100;
  det.y = 200;
  det.width = 50;
  det.height = 50;
  det_msg.detections.push_back(det);
  detection_pub_->publish(det_msg);

  // Small delay for detection to be cached.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Send TargetRequest.
  TargetRequest req_msg;
  req_msg.header.stamp = node_->now();
  req_msg.request_id = "target-req-1";
  req_msg.trace_id = "trace-1";
  req_msg.epoch = 1;
  req_msg.target_class = "cup";
  req_msg.minimum_score = 0.5f;
  req_msg.timeout_ms = 1000;
  target_request_pub_->publish(req_msg);

  // Should get a TargetResponse and return to IDLE.
  ASSERT_TRUE(resp_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_TRUE(resp_col.messages()[0].found);
  EXPECT_EQ(resp_col.messages()[0].target_class, "cup");
  EXPECT_FLOAT_EQ(resp_col.messages()[0].score, 0.95f);

  // Wait for IDLE transition.
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);
}

}  // namespace
}  // namespace k1muse_multimodal_supervisor
