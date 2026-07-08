#include <chrono>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "k1muse_manager_msgs/action/execute_task.hpp"
#include "k1muse_vision_msgs/msg/target3_d.hpp"
#include "k1muse_vision_msgs/msg/target_request.hpp"
#include "k1muse_vision_msgs/msg/target_response.hpp"

#include "k1muse_control_manager/control_manager_node.hpp"

using ExecuteTask = k1muse_manager_msgs::action::ExecuteTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;
using k1muse_vision_msgs::msg::TargetRequest;
using k1muse_vision_msgs::msg::TargetResponse;
using k1muse_vision_msgs::msg::Target3D;

static constexpr auto kWaitTimeout = std::chrono::seconds(10);

class ControlManagerNodeTest : public ::testing::Test
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
    node_ = std::make_shared<k1muse_control_manager::ControlManagerNode>();
    client_ = rclcpp_action::create_client<ExecuteTask>(
      node_, "execute_task");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  rclcpp::Node::SharedPtr make_helper_node(const std::string & name)
  {
    return std::make_shared<rclcpp::Node>(name);
  }

  GoalHandle::SharedPtr send_goal_and_wait(const std::string & task_type)
  {
    auto goal = ExecuteTask::Goal();
    goal.task_type = task_type;
    goal.trace_id = "test-trace";
    goal.epoch = 0;

    auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
    auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(node_, goal_handle_future, kWaitTimeout),
      rclcpp::FutureReturnCode::SUCCESS);
    return goal_handle_future.get();
  }

  ExecuteTask::Result::SharedPtr wait_for_result(GoalHandle::SharedPtr goal_handle)
  {
    auto result_future = client_->async_get_result(goal_handle);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(node_, result_future, kWaitTimeout),
      rclcpp::FutureReturnCode::SUCCESS);
    return result_future.get().result;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr client_;
};

TEST_F(ControlManagerNodeTest, MoveGoalAccepted)
{
  auto goal_handle = send_goal_and_wait("move");
  ASSERT_NE(goal_handle, nullptr);

  auto result = wait_for_result(goal_handle);
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->success);
  EXPECT_EQ(result->final_state, 2u);  // STATE_SUCCEEDED
}

TEST_F(ControlManagerNodeTest, PickPlaceGoalsAcceptedInMockBackend)
{
  auto pick_handle = send_goal_and_wait("pick");
  ASSERT_NE(pick_handle, nullptr);

  auto pick_result = wait_for_result(pick_handle);
  ASSERT_NE(pick_result, nullptr);
  EXPECT_TRUE(pick_result->success);
  EXPECT_EQ(pick_result->final_state, 2u);  // STATE_SUCCEEDED

  auto place_handle = send_goal_and_wait("place");
  ASSERT_NE(place_handle, nullptr);

  auto place_result = wait_for_result(place_handle);
  ASSERT_NE(place_result, nullptr);
  EXPECT_TRUE(place_result->success);
  EXPECT_EQ(place_result->final_state, 2u);  // STATE_SUCCEEDED

  auto pick_place_handle = send_goal_and_wait("pick_place");
  ASSERT_NE(pick_place_handle, nullptr);

  auto pick_place_result = wait_for_result(pick_place_handle);
  ASSERT_NE(pick_place_result, nullptr);
  EXPECT_TRUE(pick_place_result->success);
  EXPECT_EQ(pick_place_result->final_state, 2u);  // STATE_SUCCEEDED
}

TEST_F(ControlManagerNodeTest, FindGoalWithTarget)
{
  // Helper node to simulate vision system
  auto helper = make_helper_node("vision_helper");

  auto request_sub = helper->create_subscription<TargetRequest>(
    "/vision/target_request", rclcpp::QoS(5).reliable().transient_local(),
    [&helper](TargetRequest::SharedPtr msg) {
      // Publish matching TargetResponse with found=true
      auto resp_pub = helper->create_publisher<TargetResponse>(
        "/vision/target_response", rclcpp::QoS(5).reliable().transient_local());
      TargetResponse resp;
      resp.header.stamp = helper->now();
      resp.request_id = msg->request_id;
      resp.trace_id = msg->trace_id;
      resp.epoch = msg->epoch;
      resp.found = true;
      resp.target_id = "target-1";
      resp.target_class = msg->target_class;
      resp.score = 0.95f;
      resp.reason = "";
      resp_pub->publish(resp);

      // Publish matching Target3D with valid=true
      auto t3d_pub = helper->create_publisher<Target3D>(
        "/vision/main/target_3d", rclcpp::QoS(5).reliable().transient_local());
      Target3D t3d;
      t3d.header.stamp = helper->now();
      t3d.request_id = msg->request_id;
      t3d.trace_id = msg->trace_id;
      t3d.epoch = msg->epoch;
      t3d.target_id = "target-1";
      t3d.x = 1.5f;
      t3d.y = 2.0f;
      t3d.z = 0.5f;
      t3d.depth = 2.5f;
      t3d.valid = true;
      t3d.reason = "";
      t3d_pub->publish(t3d);
    });

  // Send find goal
  auto goal = ExecuteTask::Goal();
  goal.task_type = "find";
  goal.target_class = "cup";
  goal.trace_id = "test-trace-find";
  goal.epoch = 0;
  goal.timeout.sec = 5;

  auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
  auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);

  // Spin both nodes
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(helper);
  auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    exec.spin_some(std::chrono::milliseconds(50));
    if (goal_handle_future.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready)
    {
      break;
    }
    if (std::chrono::steady_clock::now() - start > kWaitTimeout) {
      FAIL() << "Timeout waiting for goal handle";
      break;
    }
  }

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client_->async_get_result(goal_handle);
  start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    exec.spin_some(std::chrono::milliseconds(50));
    if (result_future.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready)
    {
      break;
    }
    if (std::chrono::steady_clock::now() - start > kWaitTimeout) {
      FAIL() << "Timeout waiting for result";
      break;
    }
  }

  auto result = result_future.get().result;
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->success);
  EXPECT_EQ(result->final_state, 2u);  // STATE_SUCCEEDED
  EXPECT_NE(result->reason.find("1.5"), std::string::npos);
  EXPECT_NE(result->reason.find("2.0"), std::string::npos);
  EXPECT_NE(result->reason.find("0.5"), std::string::npos);
}

TEST_F(ControlManagerNodeTest, FindGoalNotFound)
{
  // Helper node that responds with found=false
  auto helper = make_helper_node("vision_helper_notfound");

  auto request_sub = helper->create_subscription<TargetRequest>(
    "/vision/target_request", rclcpp::QoS(5).reliable().transient_local(),
    [&helper](TargetRequest::SharedPtr msg) {
      auto resp_pub = helper->create_publisher<TargetResponse>(
        "/vision/target_response", rclcpp::QoS(5).reliable().transient_local());
      TargetResponse resp;
      resp.header.stamp = helper->now();
      resp.request_id = msg->request_id;
      resp.trace_id = msg->trace_id;
      resp.epoch = msg->epoch;
      resp.found = false;
      resp.target_id = "";
      resp.target_class = msg->target_class;
      resp.score = 0.0f;
      resp.reason = "no object detected";
      resp_pub->publish(resp);
    });

  auto goal = ExecuteTask::Goal();
  goal.task_type = "find";
  goal.target_class = "nonexistent";
  goal.trace_id = "test-trace-nf";
  goal.epoch = 0;
  goal.timeout.sec = 5;

  auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
  auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(helper);
  auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    exec.spin_some(std::chrono::milliseconds(50));
    if (goal_handle_future.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready)
    {
      break;
    }
    if (std::chrono::steady_clock::now() - start > kWaitTimeout) {
      FAIL() << "Timeout waiting for goal handle";
      break;
    }
  }

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client_->async_get_result(goal_handle);
  start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    exec.spin_some(std::chrono::milliseconds(50));
    if (result_future.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready)
    {
      break;
    }
    if (std::chrono::steady_clock::now() - start > kWaitTimeout) {
      FAIL() << "Timeout waiting for result";
      break;
    }
  }

  auto result = result_future.get().result;
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(result->final_state, 3u);  // STATE_FAILED
  EXPECT_NE(result->reason.find("not found"), std::string::npos);
}

TEST_F(ControlManagerNodeTest, CancelGoal)
{
  // Use a long mock delay so we have time to cancel
  node_->set_parameter(rclcpp::Parameter("mock_delay_ms", 5000));

  auto goal = ExecuteTask::Goal();
  goal.task_type = "move";
  goal.trace_id = "test-trace-cancel";
  goal.epoch = 0;

  auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
  auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, goal_handle_future, kWaitTimeout),
    rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Cancel the goal
  auto cancel_future = client_->async_cancel_goal(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, cancel_future, kWaitTimeout),
    rclcpp::FutureReturnCode::SUCCESS);

  // Wait for the result (should be cancelled)
  auto result_future = client_->async_get_result(goal_handle);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node_, result_future, kWaitTimeout),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = result_future.get().result;
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(result->final_state, 4u);  // STATE_CANCELLED
  EXPECT_NE(result->reason.find("cancel"), std::string::npos);
}
