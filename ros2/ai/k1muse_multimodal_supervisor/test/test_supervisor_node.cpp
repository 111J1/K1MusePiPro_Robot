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
#include "k1muse_voice_msgs/msg/audio_play_request.hpp"
#include "k1muse_voice_msgs/msg/audio_playback_state.hpp"
#include "k1muse_voice_msgs/msg/wakeword_event.hpp"

namespace k1muse_multimodal_supervisor {
namespace {

using AiRuntimeState = k1muse_ai_runtime_msgs::msg::AiRuntimeState;
using AiRuntimeControl = k1muse_ai_runtime_msgs::msg::AiRuntimeControl;
using WakewordEvent = k1muse_voice_msgs::msg::WakewordEvent;
using AudioPlayRequest = k1muse_voice_msgs::msg::AudioPlayRequest;
using AudioPlaybackState = k1muse_voice_msgs::msg::AudioPlaybackState;
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

class SupervisorTestEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

::testing::Environment* const kEnv =
    ::testing::AddGlobalTestEnvironment(new SupervisorTestEnvironment);

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class SupervisorNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("no_speech_timeout_sec", 2.0),
        rclcpp::Parameter("wake_ack_preset", std::string{"wake_ack"}),
        rclcpp::Parameter("target_cache_ttl_ms", 500),
    });
    node_ = std::make_shared<MultimodalSupervisorNode>(options);

    // Helper publishers (to send messages TO the supervisor).
    runtime_state_pub_ = node_->create_publisher<AiRuntimeState>(
        "/ai_runtime/state", rclcpp::QoS(1).reliable().transient_local());
    wakeword_pub_ = node_->create_publisher<WakewordEvent>(
        "/voice/wakeword/event", rclcpp::QoS(5).reliable().durability_volatile());
    audio_io_pub_ = node_->create_publisher<NodeReady>(
        "/audio_io/state", rclcpp::QoS(1).reliable().transient_local());
    intent_state_pub_ = node_->create_publisher<NodeReady>(
        "/intent/state", rclcpp::QoS(1).reliable().transient_local());
    playback_state_pub_ = node_->create_publisher<AudioPlaybackState>(
        "/voice/audio/playback_state", rclcpp::QoS(1).reliable().transient_local());

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

  // Publish AiRuntimeState with given readiness.
  void publish_runtime_state(bool ready) {
    AiRuntimeState msg;
    msg.header.stamp = node_->now();
    msg.runtime_ready = ready;
    msg.epoch = 0;
    runtime_state_pub_->publish(msg);
  }

  // Publish NodeReady on audio_io/state.
  void publish_audio_ready(bool ready) {
    NodeReady msg;
    msg.header.stamp = node_->now();
    msg.node_name = "audio_io";
    msg.ready = ready;
    audio_io_pub_->publish(msg);
  }

  // Publish NodeReady on intent/state.
  void publish_intent_ready(bool ready) {
    NodeReady msg;
    msg.header.stamp = node_->now();
    msg.node_name = "intent_node";
    msg.ready = ready;
    intent_state_pub_->publish(msg);
  }

  // Publish a wakeword event.
  void publish_wakeword(uint64_t epoch = 1) {
    WakewordEvent msg;
    msg.header.stamp = node_->now();
    msg.trace_id = "test_trace";
    msg.epoch = epoch;
    msg.event = WakewordEvent::EVENT_DETECTED;
    msg.keyword = "hello";
    msg.confidence = 0.95f;
    wakeword_pub_->publish(msg);
  }

  // Publish AudioPlaybackState DONE with matching identity.
  void publish_playback_done(const std::string& request_id, uint64_t epoch) {
    AudioPlaybackState msg;
    msg.header.stamp = node_->now();
    msg.request_id = request_id;
    msg.epoch = epoch;
    msg.source = "test";
    msg.state = AudioPlaybackState::STATE_DONE;
    msg.state_name = "DONE";
    playback_state_pub_->publish(msg);
  }

  std::shared_ptr<MultimodalSupervisorNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::future<void> spin_future_;

  rclcpp::Publisher<AiRuntimeState>::SharedPtr runtime_state_pub_;
  rclcpp::Publisher<WakewordEvent>::SharedPtr wakeword_pub_;
  rclcpp::Publisher<NodeReady>::SharedPtr audio_io_pub_;
  rclcpp::Publisher<NodeReady>::SharedPtr intent_state_pub_;
  rclcpp::Publisher<AudioPlaybackState>::SharedPtr playback_state_pub_;
};

// ---------------------------------------------------------------------------
// Test 1: BootWaitsForReadiness
// Not all subsystems ready -> stays in BOOT.
// ---------------------------------------------------------------------------

TEST_F(SupervisorNodeTest, BootWaitsForReadiness) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(1).reliable().transient_local());

  // Publish only runtime ready, not audio/intent.
  publish_runtime_state(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Should still be in BOOT.
  EXPECT_EQ(node_->current_state(), State::BOOT);
}

// ---------------------------------------------------------------------------
// Test 2: BootToIdle
// All three ready -> transitions to IDLE, control published.
// ---------------------------------------------------------------------------

TEST_F(SupervisorNodeTest, BootToIdle) {
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());

  publish_audio_ready(true);
  publish_intent_ready(true);
  publish_runtime_state(true);

  // Wait for IDLE state.
  ASSERT_TRUE(ctrl_col.wait_for(2, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::IDLE);

  // Verify last control message has IDLE flags.
  const auto& last = ctrl_col.messages().back();
  EXPECT_EQ(last.interaction_state, static_cast<uint8_t>(State::IDLE));
  EXPECT_TRUE(last.wakeword_enabled);
  EXPECT_TRUE(last.vision_enabled);
  EXPECT_FALSE(last.vad_asr_enabled);
  EXPECT_FALSE(last.tts_enabled);
}

// ---------------------------------------------------------------------------
// Test 3: IdleToWakeAck
// Wakeword event -> WAKE_ACK, AudioPlayRequest published.
// ---------------------------------------------------------------------------

TEST_F(SupervisorNodeTest, IdleToWakeAck) {
  // Get to IDLE first.
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());

  publish_audio_ready(true);
  publish_intent_ready(true);
  publish_runtime_state(true);
  ASSERT_TRUE(ctrl_col.wait_for(2, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::IDLE);

  // Now listen for AudioPlayRequest.
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  publish_wakeword();

  ASSERT_TRUE(play_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_EQ(play_col.messages()[0].preset_name, "wake_ack");
  EXPECT_FALSE(play_col.messages()[0].request_id.empty());

  // Wait for WAKE_ACK control message.
  ASSERT_TRUE(ctrl_col.wait_for(3, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::WAKE_ACK);
}

// ---------------------------------------------------------------------------
// Test 4: WakeAckToListening
// AudioPlaybackState DONE with matching request_id -> LISTENING.
// ---------------------------------------------------------------------------

TEST_F(SupervisorNodeTest, WakeAckToListening) {
  // Get to IDLE.
  MessageCollector<AiRuntimeControl> ctrl_col(
      node_, "/ai_runtime/control",
      rclcpp::QoS(10).reliable().transient_local());
  MessageCollector<AudioPlayRequest> play_col(
      node_, "/voice/audio/play",
      rclcpp::QoS(5).reliable().durability_volatile());

  publish_audio_ready(true);
  publish_intent_ready(true);
  publish_runtime_state(true);
  ASSERT_TRUE(ctrl_col.wait_for(2, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::IDLE);

  // Trigger wakeword.
  publish_wakeword(42);
  ASSERT_TRUE(play_col.wait_for(1, std::chrono::milliseconds(3000)));
  ASSERT_EQ(node_->current_state(), State::WAKE_ACK);

  // Get the request_id from the published AudioPlayRequest.
  std::string req_id = play_col.messages()[0].request_id;
  uint64_t epoch = play_col.messages()[0].epoch;

  // Publish playback DONE with matching identity.
  publish_playback_done(req_id, epoch);

  // Wait for LISTENING.
  ASSERT_TRUE(ctrl_col.wait_for(4, std::chrono::milliseconds(3000)));
  EXPECT_EQ(node_->current_state(), State::LISTENING);
}

}  // namespace
}  // namespace k1muse_multimodal_supervisor
