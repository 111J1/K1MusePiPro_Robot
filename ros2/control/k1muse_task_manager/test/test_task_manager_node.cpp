#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "k1muse_manager_msgs/action/execute_task.hpp"
#include "k1muse_manager_msgs/msg/task_status.hpp"
#include "k1muse_task_manager/task_manager_node.hpp"
#include "k1muse_voice_msgs/msg/intent_lite.hpp"

using namespace std::chrono_literals;

using ExecuteTask = k1muse_manager_msgs::action::ExecuteTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
using IntentLite = k1muse_voice_msgs::msg::IntentLite;
using TaskStatus = k1muse_manager_msgs::msg::TaskStatus;

class TaskManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    // Test helper node for pub/sub and mock action server
    test_node_ = std::make_shared<rclcpp::Node>("test_helper_node");

    // Mock action server
    mock_server_ = rclcpp_action::create_server<ExecuteTask>(
      test_node_,
      "execute_task",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal) {
        last_goal_ = *goal;
        goal_received_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        // Hold the goal handle for manual result triggering
        std::lock_guard<std::mutex> lock(goal_mutex_);
        active_goal_handle_ = goal_handle;
      });

    // Publisher for IntentLite
    intent_pub_ = test_node_->create_publisher<IntentLite>("/voice/intent", 10);

    // Subscription for TaskStatus
    status_sub_ = test_node_->create_subscription<TaskStatus>(
      "/manager/task_status", 10,
      [this](TaskStatus::SharedPtr msg) {
        received_statuses_.push_back(*msg);
      });

    // TaskManagerNode under test
    task_manager_ = std::make_shared<k1muse_task_manager::TaskManagerNode>(
      rclcpp::NodeOptions());

    // Multi-threaded executor for both nodes
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(test_node_);
    executor_->add_node(task_manager_);
  }

  void TearDown() override
  {
    executor_->cancel();
    task_manager_.reset();
    test_node_.reset();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_once(100ms);
    }
  }

  void complete_goal_success()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_handle_) {
      auto result = std::make_shared<ExecuteTask::Result>();
      result->trace_id = last_goal_.trace_id;
      result->request_id = last_goal_.request_id;
      result->task_id = last_goal_.task_id;
      result->epoch = last_goal_.epoch;
      result->success = true;
      result->final_state = TaskStatus::STATE_SUCCEEDED;
      result->reason = "test completed";
      active_goal_handle_->succeed(result);
    }
  }

  IntentLite make_intent(const std::string & intent_name,
                         const std::string & action,
                         const std::string & target = "",
                         const std::string & value = "")
  {
    IntentLite msg;
    msg.header.stamp = test_node_->now();
    msg.trace_id = "test-trace-001";
    msg.request_id = "test-req-001";
    msg.epoch = 12345;
    msg.intent_name = intent_name;
    msg.action = action;
    msg.target = target;
    msg.value = value;
    return msg;
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<k1muse_task_manager::TaskManagerNode> task_manager_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr mock_server_;
  rclcpp::Publisher<IntentLite>::SharedPtr intent_pub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  // Mock server state
  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandle> active_goal_handle_;
  ExecuteTask::Goal last_goal_;
  bool goal_received_ = false;

  // Received status messages
  std::vector<TaskStatus> received_statuses_;
};

TEST_F(TaskManagerTest, IntentTriggersGoal)
{
  auto intent = make_intent("action", "move", "", "1.0");
  intent_pub_->publish(intent);

  spin_for(2000ms);

  EXPECT_TRUE(goal_received_);
  EXPECT_EQ(last_goal_.task_type, "move");
  EXPECT_EQ(last_goal_.trace_id, "test-trace-001");
  EXPECT_EQ(last_goal_.request_id, "test-req-001");
  EXPECT_EQ(last_goal_.epoch, 12345u);
  EXPECT_FALSE(last_goal_.task_id.empty());
  EXPECT_EQ(last_goal_.target_class, "1.0");
}

TEST_F(TaskManagerTest, StopCancelsGoal)
{
  // Send move intent first
  auto move_intent = make_intent("action", "move", "", "1.0");
  intent_pub_->publish(move_intent);
  spin_for(2000ms);
  EXPECT_TRUE(goal_received_);

  // Reset for stop
  goal_received_ = false;

  // Send stop intent
  auto stop_intent = make_intent("action", "stop");
  intent_pub_->publish(stop_intent);
  spin_for(2000ms);

  // Stop should not create a new goal
  EXPECT_FALSE(goal_received_);
}

TEST_F(TaskManagerTest, TaskStatusPublished)
{
  auto intent = make_intent("action", "find", "cup");
  intent_pub_->publish(intent);

  spin_for(2000ms);
  EXPECT_TRUE(goal_received_);

  // Check that STATE_ACTIVE was published
  bool found_active = false;
  for (const auto & status : received_statuses_) {
    if (status.state == TaskStatus::STATE_ACTIVE) {
      found_active = true;
      EXPECT_EQ(status.trace_id, "test-trace-001");
      EXPECT_EQ(status.request_id, "test-req-001");
      EXPECT_FALSE(status.task_id.empty());
      EXPECT_EQ(status.epoch, 12345u);
      break;
    }
  }
  EXPECT_TRUE(found_active);

  // Complete the goal and check SUCCEEDED status
  complete_goal_success();
  spin_for(2000ms);

  bool found_succeeded = false;
  for (const auto & status : received_statuses_) {
    if (status.state == TaskStatus::STATE_SUCCEEDED) {
      found_succeeded = true;
      EXPECT_EQ(status.trace_id, "test-trace-001");
      break;
    }
  }
  EXPECT_TRUE(found_succeeded);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
