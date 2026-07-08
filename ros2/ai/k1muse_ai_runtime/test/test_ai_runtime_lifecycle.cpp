#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "k1muse_ai_runtime/ai_runtime_node.hpp"
#include "k1muse_ai_runtime_msgs/msg/ai_runtime_control.hpp"
#include "k1muse_common/qos_profiles.hpp"

using k1muse_ai_runtime::AiRuntimeNode;

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

TEST_F(RosContext, TwoLifecycleRoundsAndReadyOnlyWhileActive)
{
  auto node = std::make_shared<AiRuntimeNode>();
  for (int round = 0; round < 2; ++round) {
    EXPECT_EQ(
      node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_FALSE(node->runtime_ready());
    EXPECT_EQ(node->retained_model_count(), 4U);
    EXPECT_EQ(node->cpu_worker_count(), 5U);
    EXPECT_EQ(
      node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_TRUE(node->runtime_ready());
    EXPECT_EQ(
      node->deactivate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_FALSE(node->runtime_ready());
    EXPECT_EQ(node->retained_model_count(), 4U);
    EXPECT_EQ(
      node->cleanup().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(node->retained_model_count(), 0U);
    EXPECT_EQ(node->cpu_worker_count(), 0U);
  }
}

TEST_F(RosContext, InvalidConfigurationFailsWithSpecificLastError)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("asr_provider", "gpu")});
  auto node = std::make_shared<AiRuntimeNode>(options);
  EXPECT_EQ(
    node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_NE(node->last_error().find("asr_provider"), std::string::npos);
}

TEST_F(RosContext, ControlOnlyUpdatesSnapshot)
{
  auto node = std::make_shared<AiRuntimeNode>();
  ASSERT_EQ(
    node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  k1muse_ai_runtime_msgs::msg::AiRuntimeControl message;
  message.trace_id = "trace-control";
  message.epoch = 42;
  message.wakeword_enabled = true;
  message.vision_enabled = false;
  message.vad_asr_enabled = true;
  message.tts_enabled = false;
  EXPECT_TRUE(node->update_control_snapshot(message));
  const auto snapshot = node->control_snapshot();
  EXPECT_EQ(snapshot.trace_id, "trace-control");
  EXPECT_EQ(snapshot.epoch, 42U);
  EXPECT_TRUE(snapshot.wakeword_enabled);
  EXPECT_TRUE(snapshot.vad_asr_enabled);
  EXPECT_EQ(node->scheduler_stats().submitted, 0U);
  node->cleanup();
}

TEST_F(RosContext, StaleControlIsRejected)
{
  auto node = std::make_shared<AiRuntimeNode>();
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  k1muse_ai_runtime_msgs::msg::AiRuntimeControl current;
  current.epoch = 9;
  current.trace_id = "current";
  EXPECT_TRUE(node->update_control_snapshot(current));
  auto stale = current;
  stale.epoch = 8;
  stale.trace_id = "stale";
  EXPECT_FALSE(node->update_control_snapshot(stale));
  EXPECT_EQ(node->control_snapshot().trace_id, "current");
  EXPECT_NE(node->last_error().find("stale control"), std::string::npos);
  node->cleanup();
}

TEST_F(RosContext, SameEpochControlUpdateIsAccepted)
{
  auto node = std::make_shared<AiRuntimeNode>();
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  k1muse_ai_runtime_msgs::msg::AiRuntimeControl first;
  first.epoch = 4;
  first.vision_enabled = false;
  EXPECT_TRUE(node->update_control_snapshot(first));
  auto update = first;
  update.vision_enabled = true;
  EXPECT_TRUE(node->update_control_snapshot(update));
  EXPECT_TRUE(node->control_snapshot().vision_enabled);
  node->cleanup();
}

TEST_F(RosContext, ConfiguredStatePublishesNotReady)
{
  auto node = std::make_shared<AiRuntimeNode>();
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  const auto state = node->last_published_state();
  EXPECT_FALSE(state.runtime_ready);
  EXPECT_EQ(state.lifecycle_state, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  node->cleanup();
}

TEST_F(RosContext, DeactivateFinalStateRemainsInactive)
{
  auto node = std::make_shared<AiRuntimeNode>();
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  const auto state = node->last_published_state();
  EXPECT_FALSE(state.runtime_ready);
  EXPECT_EQ(state.lifecycle_state, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(state.lifecycle_state_name, "inactive");
  node->cleanup();
}

TEST_F(RosContext, InvalidConfigurePublishesPersistentNotReadyReason)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("guarded_queue_capacity", 0)});
  auto node = std::make_shared<AiRuntimeNode>(options);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  const auto state = node->last_published_state();
  EXPECT_FALSE(state.runtime_ready);
  EXPECT_NE(state.last_error.find("guarded_queue_capacity"), std::string::npos);
}

TEST_F(RosContext, TransientLocalLateJoinerReceivesInvalidConfigureReason)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("guarded_queue_capacity", 0)});
  auto runtime = std::make_shared<AiRuntimeNode>(options);
  EXPECT_EQ(runtime->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto observer = std::make_shared<rclcpp::Node>("invalid_config_observer");
  std::mutex mutex;
  std::condition_variable condition;
  std::optional<k1muse_ai_runtime_msgs::msg::AiRuntimeState> received;
  auto subscription =
    observer->create_subscription<k1muse_ai_runtime_msgs::msg::AiRuntimeState>(
    "/ai_runtime/state", k1muse_common::qos::LatchedState(),
    [&](k1muse_ai_runtime_msgs::msg::AiRuntimeState::SharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        received = *message;
      }
      condition.notify_all();
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(runtime->get_node_base_interface());
  executor.add_node(observer);
  std::thread spin([&]() {executor.spin();});
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&]() {
      return received.has_value();
    }));
  }
  executor.cancel();
  spin.join();
  ASSERT_TRUE(received.has_value());
  EXPECT_FALSE(received->runtime_ready);
  EXPECT_NE(received->last_error.find("guarded_queue_capacity"), std::string::npos);
  (void)subscription;
}

TEST_F(RosContext, TransientLocalLateJoinerReceivesCleanupState)
{
  auto runtime = std::make_shared<AiRuntimeNode>();
  ASSERT_EQ(runtime->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(runtime->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(runtime->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(runtime->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto observer = std::make_shared<rclcpp::Node>("late_state_observer");
  std::mutex mutex;
  std::condition_variable condition;
  std::optional<k1muse_ai_runtime_msgs::msg::AiRuntimeState> received;
  auto subscription =
    observer->create_subscription<k1muse_ai_runtime_msgs::msg::AiRuntimeState>(
    "/ai_runtime/state", k1muse_common::qos::LatchedState(),
    [&](k1muse_ai_runtime_msgs::msg::AiRuntimeState::SharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        received = *message;
      }
      condition.notify_all();
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(runtime->get_node_base_interface());
  executor.add_node(observer);
  std::thread spin([&]() {executor.spin();});
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&]() {
      return received.has_value();
    }));
  }
  executor.cancel();
  spin.join();
  ASSERT_TRUE(received.has_value());
  EXPECT_FALSE(received->runtime_ready);
  EXPECT_EQ(
    received->lifecycle_state,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(RosContext, ExecutorSubscriberObservesActiveDeactivateCleanupStates)
{
  auto runtime = std::make_shared<AiRuntimeNode>();
  auto observer = std::make_shared<rclcpp::Node>("state_sequence_observer");
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<k1muse_ai_runtime_msgs::msg::AiRuntimeState> states;
  auto subscription =
    observer->create_subscription<k1muse_ai_runtime_msgs::msg::AiRuntimeState>(
    "/ai_runtime/state", k1muse_common::qos::LatchedState(),
    [&](k1muse_ai_runtime_msgs::msg::AiRuntimeState::SharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        states.push_back(*message);
      }
      condition.notify_all();
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(runtime->get_node_base_interface());
  executor.add_node(observer);
  std::thread spin([&]() {executor.spin();});

  const auto wait_for_state = [&](uint8_t lifecycle_state, bool ready) {
      std::unique_lock<std::mutex> lock(mutex);
      return condition.wait_for(lock, std::chrono::seconds(1), [&]() {
        return std::any_of(
          states.begin(), states.end(),
          [=](const auto & state) {
            return state.lifecycle_state == lifecycle_state &&
                   state.runtime_ready == ready;
          });
      });
    };

  EXPECT_EQ(runtime->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(runtime->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, true));
  EXPECT_EQ(runtime->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, false));
  EXPECT_EQ(runtime->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, false));

  executor.cancel();
  spin.join();
  (void)subscription;
}

TEST_F(RosContext, ExecutorSubscriberObservesErrorFinalNotReady)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("mock_warmup_failure", true)});
  auto runtime = std::make_shared<AiRuntimeNode>(options);
  auto observer = std::make_shared<rclcpp::Node>("error_state_observer");
  std::mutex mutex;
  std::condition_variable condition;
  std::optional<k1muse_ai_runtime_msgs::msg::AiRuntimeState> final_error;
  auto subscription =
    observer->create_subscription<k1muse_ai_runtime_msgs::msg::AiRuntimeState>(
    "/ai_runtime/state", k1muse_common::qos::LatchedState(),
    [&](k1muse_ai_runtime_msgs::msg::AiRuntimeState::SharedPtr message) {
      if (!message->runtime_ready &&
        message->lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
        message->last_error.find("warmup") != std::string::npos)
      {
        {
          std::lock_guard<std::mutex> lock(mutex);
          final_error = *message;
        }
        condition.notify_all();
      }
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(runtime->get_node_base_interface());
  executor.add_node(observer);
  std::thread spin([&]() {executor.spin();});

  EXPECT_EQ(runtime->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(runtime->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&]() {
      return final_error.has_value();
    }));
  }
  executor.cancel();
  spin.join();
  ASSERT_TRUE(final_error.has_value());
  EXPECT_FALSE(final_error->runtime_ready);
  (void)subscription;
}

TEST_F(RosContext, WarmupFailureEntersErrorAndCanReconfigure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("mock_warmup_failure", true)});
  auto node = std::make_shared<AiRuntimeNode>(options);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(node->runtime_ready());
  EXPECT_EQ(node->retained_model_count(), 0U);
  EXPECT_EQ(
    node->last_published_state().lifecycle_state,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node->set_parameter(rclcpp::Parameter("mock_warmup_failure", false));
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  node->deactivate();
  node->cleanup();
}

TEST_F(RosContext, InactiveShutdownPublishesNotReady)
{
  auto node = std::make_shared<AiRuntimeNode>();
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_FALSE(node->last_published_state().runtime_ready);
  EXPECT_EQ(
    node->last_published_state().lifecycle_state,
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_EQ(node->retained_model_count(), 0U);
}
