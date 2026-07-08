#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "k1muse_manager_msgs/action/execute_task.hpp"
#include "k1muse_manager_msgs/msg/task_status.hpp"
#include "k1muse_voice_msgs/msg/intent_lite.hpp"

namespace k1muse_task_manager
{

using ExecuteTask = k1muse_manager_msgs::action::ExecuteTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;

class TaskManagerNode : public rclcpp::Node
{
public:
  explicit TaskManagerNode(const rclcpp::NodeOptions & options);

private:
  void on_intent(k1muse_voice_msgs::msg::IntentLite::SharedPtr msg);
  void send_goal(const k1muse_voice_msgs::msg::IntentLite & intent);
  void cancel_active_goal();
  void publish_status(
    const std::string & trace_id,
    const std::string & request_id,
    const std::string & task_id,
    uint64_t epoch,
    uint8_t state,
    const std::string & state_name,
    float progress,
    const std::string & reason);

  rclcpp::Subscription<k1muse_voice_msgs::msg::IntentLite>::SharedPtr intent_subscription_;
  rclcpp::Publisher<k1muse_manager_msgs::msg::TaskStatus>::SharedPtr status_publisher_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr action_client_;

  // Active goal tracking
  std::mutex active_mutex_;
  GoalHandle::SharedPtr active_goal_;
  std::string active_task_id_;
  std::string active_trace_id_;
  uint64_t active_epoch_ = 0;

  int default_timeout_ms_ = 5000;
};

}  // namespace k1muse_task_manager
