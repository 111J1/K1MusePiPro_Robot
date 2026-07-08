#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "k1muse_common/qos_profiles.hpp"
#include "k1muse_voice_audio/audio_device.hpp"
#include "k1muse_voice_audio/audio_io_node.hpp"
#include "k1muse_voice_audio/mock_audio_device.hpp"
#include "k1muse_voice_msgs/msg/audio_play_request.hpp"
#include "k1muse_voice_msgs/msg/audio_playback_state.hpp"
#include "k1muse_voice_msgs/msg/tts_play_request.hpp"
#include "k1muse_common/msg/node_ready.hpp"

using k1muse_voice_audio::AudioDevice;
using k1muse_voice_audio::AudioIoNode;
using k1muse_voice_audio::MockAudioDevice;
using AudioPlaybackStateMessage = k1muse_voice_msgs::msg::AudioPlaybackState;
using AudioPlayRequestMessage = k1muse_voice_msgs::msg::AudioPlayRequest;
using TtsPlayRequestMessage = k1muse_voice_msgs::msg::TtsPlayRequest;
using NodeReadyMessage = k1muse_common::msg::NodeReady;

// ---------------------------------------------------------------------------
// Test fixture: manages rclcpp lifecycle
// ---------------------------------------------------------------------------
class RosContext : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// ---------------------------------------------------------------------------
// ControllableMockAudioDevice -- allows blocking play() via a caller-supplied
// gate so the playback worker can be held in the PLAYING state while additional
// requests arrive.
// ---------------------------------------------------------------------------
class ControllableMockAudioDevice : public AudioDevice
{
public:
  // Call from the test thread to release a blocked play() call.
  void release()
  {
    std::lock_guard<std::mutex> lk(mu_);
    released_ = true;
    cv_.notify_all();
  }

  const std::string & name() const override { return name_; }
  void load() override { loaded_ = true; }
  void unload() override { loaded_ = false; }
  bool loaded() const override { return loaded_; }

  PlaybackResult play_pcm(
    const std::vector<int16_t> & /*pcm*/, uint32_t /*sample_rate*/,
    uint8_t /*channels*/, const std::string & /*encoding*/) override
  {
    block_until_released();
    return {true, std::chrono::milliseconds{0}, {}};
  }

  PlaybackResult play_preset(const std::string & /*preset_name*/) override
  {
    block_until_released();
    return {true, std::chrono::milliseconds{0}, {}};
  }

  void stop() override
  {
    std::lock_guard<std::mutex> lk(mu_);
    released_ = true;
    cv_.notify_all();
  }

private:
  void block_until_released()
  {
    std::unique_lock<std::mutex> lk(mu_);
    cv_.wait(lk, [this]() { return released_; });
    // Reset for potential reuse (not strictly needed for these tests).
    released_ = false;
  }

  std::string name_{"controllable_mock"};
  bool loaded_{false};
  std::mutex mu_;
  std::condition_variable cv_;
  bool released_{false};
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static std::shared_ptr<AudioIoNode> make_active_node()
{
  auto node = std::make_shared<AudioIoNode>();
  EXPECT_EQ(
    node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    node->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  return node;
}

static TtsPlayRequestMessage make_tts_request(
  const std::string & trace_id = "trace-1",
  const std::string & request_id = "req-1",
  uint64_t epoch = 1,
  std::size_t pcm_samples = 1600)
{
  TtsPlayRequestMessage msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.trace_id = trace_id;
  msg.request_id = request_id;
  msg.epoch = epoch;
  msg.source = "test";
  msg.sample_rate = 16000;
  msg.channels = 1;
  msg.encoding = "pcm_s16le";
  msg.pcm_s16le = std::vector<int16_t>(pcm_samples, 0);
  return msg;
}

static AudioPlayRequestMessage make_preset_request(
  const std::string & preset_name,
  const std::string & trace_id = "trace-1",
  const std::string & request_id = "req-1",
  uint64_t epoch = 1)
{
  AudioPlayRequestMessage msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.trace_id = trace_id;
  msg.request_id = request_id;
  msg.epoch = epoch;
  msg.source = "test";
  msg.kind = AudioPlayRequestMessage::KIND_PRESET;
  msg.preset_name = preset_name;
  return msg;
}

// ---------------------------------------------------------------------------
// Test 1: PlaybackQueueSerial
// Submit two requests sequentially. Both should be processed one after another
// (the second is queued while the first plays, then processed).
// ---------------------------------------------------------------------------
TEST_F(RosContext, PlaybackQueueSerial)
{
  auto node = make_active_node();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  std::thread spin([&]() { executor.spin(); });

  auto observer = std::make_shared<rclcpp::Node>("serial_observer");
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<AudioPlaybackStateMessage> done_states;

  auto sub = observer->create_subscription<AudioPlaybackStateMessage>(
    "/voice/audio/playback_state", k1muse_common::qos::LatchedState(1),
    [&](AudioPlaybackStateMessage::SharedPtr msg) {
      if (msg->state == AudioPlaybackStateMessage::STATE_DONE) {
        std::lock_guard<std::mutex> lock(mutex);
        done_states.push_back(*msg);
        cv.notify_all();
      }
    });
  executor.add_node(observer);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto publisher = std::make_shared<rclcpp::Node>("serial_pub");
  auto tts_pub = publisher->create_publisher<TtsPlayRequestMessage>(
    "/voice/tts/play", k1muse_common::qos::ReliableEvent(5));
  executor.add_node(publisher);

  // First request: 100ms PCM
  auto req1 = make_tts_request("trace-1", "req-1", 1, 1600);
  tts_pub->publish(req1);

  // Wait for the worker to pick up the first request
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Second request: should be enqueued and processed after first completes
  auto req2 = make_tts_request("trace-2", "req-2", 2, 1600);
  tts_pub->publish(req2);

  // Wait for both to complete
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cv.wait_for(lock, std::chrono::seconds(5), [&]() {
      return done_states.size() >= 2;
    }));
  }

  ASSERT_GE(done_states.size(), 2U);
  EXPECT_EQ(node->queue_size(), 0U);

  executor.cancel();
  spin.join();
  node->deactivate();
  node->cleanup();
}

// ---------------------------------------------------------------------------
// Test 2: PlaybackStateTransitions
// Verify PENDING -> PLAYING -> DONE for a single request.
// ---------------------------------------------------------------------------
TEST_F(RosContext, PlaybackStateTransitions)
{
  auto node = make_active_node();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  std::thread spin([&]() { executor.spin(); });

  auto observer = std::make_shared<rclcpp::Node>("state_observer");
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<uint8_t> states;

  auto sub = observer->create_subscription<AudioPlaybackStateMessage>(
    "/voice/audio/playback_state", k1muse_common::qos::LatchedState(1),
    [&](AudioPlaybackStateMessage::SharedPtr msg) {
      if (msg->request_id == "req-state") {
        std::lock_guard<std::mutex> lock(mutex);
        states.push_back(msg->state);
        cv.notify_all();
      }
    });
  executor.add_node(observer);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto publisher = std::make_shared<rclcpp::Node>("test_pub2");
  auto tts_pub = publisher->create_publisher<TtsPlayRequestMessage>(
    "/voice/tts/play", k1muse_common::qos::ReliableEvent(5));
  executor.add_node(publisher);

  // Short PCM request (10ms at 16kHz)
  auto req = make_tts_request("trace-state", "req-state", 1, 160);
  tts_pub->publish(req);

  // Wait for DONE
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cv.wait_for(lock, std::chrono::seconds(3), [&]() {
      return std::find(states.begin(), states.end(),
        AudioPlaybackStateMessage::STATE_DONE) != states.end();
    }));
  }

  ASSERT_GE(states.size(), 3U);
  EXPECT_EQ(states[0], AudioPlaybackStateMessage::STATE_PENDING);
  EXPECT_EQ(states[1], AudioPlaybackStateMessage::STATE_PLAYING);
  EXPECT_EQ(states[2], AudioPlaybackStateMessage::STATE_DONE);

  executor.cancel();
  spin.join();
  node->deactivate();
  node->cleanup();
}

// ---------------------------------------------------------------------------
// Test 3: PresetPlayback
// Play wake_ack preset and verify DONE is published.
// ---------------------------------------------------------------------------
TEST_F(RosContext, PresetPlayback)
{
  auto node = make_active_node();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  std::thread spin([&]() { executor.spin(); });

  auto observer = std::make_shared<rclcpp::Node>("preset_observer");
  std::mutex mutex;
  std::condition_variable cv;
  std::optional<AudioPlaybackStateMessage> done_state;

  auto sub = observer->create_subscription<AudioPlaybackStateMessage>(
    "/voice/audio/playback_state", k1muse_common::qos::LatchedState(1),
    [&](AudioPlaybackStateMessage::SharedPtr msg) {
      if (msg->request_id == "req-preset" &&
        msg->state == AudioPlaybackStateMessage::STATE_DONE)
      {
        std::lock_guard<std::mutex> lock(mutex);
        done_state = *msg;
        cv.notify_all();
      }
    });
  executor.add_node(observer);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto publisher = std::make_shared<rclcpp::Node>("preset_pub");
  auto audio_pub = publisher->create_publisher<AudioPlayRequestMessage>(
    "/voice/audio/play", k1muse_common::qos::ReliableEvent(5));
  executor.add_node(publisher);

  auto req = make_preset_request("wake_ack", "trace-preset", "req-preset", 1);
  audio_pub->publish(req);

  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cv.wait_for(lock, std::chrono::seconds(3), [&]() {
      return done_state.has_value();
    }));
  }

  ASSERT_TRUE(done_state.has_value());
  EXPECT_EQ(done_state->state, AudioPlaybackStateMessage::STATE_DONE);
  EXPECT_EQ(done_state->state_name, "DONE");
  EXPECT_EQ(done_state->trace_id, "trace-preset");

  executor.cancel();
  spin.join();
  node->deactivate();
  node->cleanup();
}

// ---------------------------------------------------------------------------
// Test 4: RejectWhenFull
// Queue capacity is 1. We inject a ControllableMockAudioDevice whose play()
// blocks until we explicitly release it. While the worker is stuck playing
// the first request (queue empty, worker busy), a second request is enqueued
// into the now-empty queue. When a third request arrives, the queue is full
// (capacity 1, second request waiting) and it must be rejected with FAILED.
// ---------------------------------------------------------------------------
TEST_F(RosContext, RejectWhenFull)
{
  auto node = std::make_shared<AudioIoNode>();
  ASSERT_EQ(
    node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Inject a controllable device so we can block playback.
  auto device = std::make_unique<ControllableMockAudioDevice>();
  device->load();
  auto * device_ptr = device.get();
  node->set_device_for_testing(std::move(device));

  ASSERT_EQ(
    node->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  std::thread spin([&]() { executor.spin(); });

  // Observe FAILED states
  auto observer = std::make_shared<rclcpp::Node>("reject_observer");
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<AudioPlaybackStateMessage> failed_states;

  auto sub = observer->create_subscription<AudioPlaybackStateMessage>(
    "/voice/audio/playback_state", k1muse_common::qos::LatchedState(1),
    [&](AudioPlaybackStateMessage::SharedPtr msg) {
      if (msg->state == AudioPlaybackStateMessage::STATE_FAILED) {
        std::lock_guard<std::mutex> lock(mutex);
        failed_states.push_back(*msg);
        cv.notify_all();
      }
    });
  executor.add_node(observer);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto publisher = std::make_shared<rclcpp::Node>("reject_pub");
  auto tts_pub = publisher->create_publisher<TtsPlayRequestMessage>(
    "/voice/tts/play", k1muse_common::qos::ReliableEvent(5));
  executor.add_node(publisher);

  // Request 1: will be dequeued by the worker, which then blocks on play().
  auto req1 = make_tts_request("trace-1", "req-fill", 1, 1600);
  tts_pub->publish(req1);

  // Give the worker time to dequeue req1 and enter play().
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Request 2: queue is empty (worker dequeued req1), so this enqueues OK.
  // The queue now has 1 item.
  auto req2 = make_tts_request("trace-2", "req-queued", 2, 1600);
  tts_pub->publish(req2);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Request 3: queue is full (req2 sitting there, worker still blocked).
  // This must be rejected with FAILED / "queue full".
  auto req3 = make_tts_request("trace-3", "req-reject", 3, 1600);
  tts_pub->publish(req3);

  // Wait for the FAILED rejection.
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cv.wait_for(lock, std::chrono::seconds(3), [&]() {
      return !failed_states.empty();
    }));
  }

  ASSERT_FALSE(failed_states.empty());
  bool found_reject = false;
  for (const auto & fs : failed_states) {
    if (fs.request_id == "req-reject") {
      found_reject = true;
      EXPECT_EQ(fs.state, AudioPlaybackStateMessage::STATE_FAILED);
      EXPECT_EQ(fs.state_name, "FAILED");
      EXPECT_EQ(fs.reason, "queue full");
    }
  }
  EXPECT_TRUE(found_reject) << "Expected req-reject to be rejected";

  // Release the device so the worker can finish processing req1 and req2.
  device_ptr->release();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // Release again for req2.
  device_ptr->release();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  executor.cancel();
  spin.join();
  node->deactivate();
  node->cleanup();
}
