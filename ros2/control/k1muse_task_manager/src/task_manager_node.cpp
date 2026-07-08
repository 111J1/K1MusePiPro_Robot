#include "k1muse_task_manager/task_manager_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "k1muse_common/id_utils.hpp"
#include "k1muse_common/qos_profiles.hpp"

namespace k1muse_task_manager
{

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: Node("task_manager_node", options)
{
  // Declare parameters
  default_timeout_ms_ = declare_parameter<int>("default_timeout_ms", 5000);

  // Action client for ExecuteTask
  action_client_ = rclcpp_action::create_client<ExecuteTask>(this, "execute_task");

  // Subscribe to /voice/intent
  intent_subscription_ = create_subscription<k1muse_voice_msgs::msg::IntentLite>(
    "/voice/intent", k1muse_common::qos::ReliableResult(10),
    std::bind(&TaskManagerNode::on_intent, this, std::placeholders::_1));

  // Publisher for TaskStatus
  status_publisher_ = create_publisher<k1muse_manager_msgs::msg::TaskStatus>(
    "/manager/task_status", k1muse_common::qos::ReliableEvent());

  RCLCPP_INFO(get_logger(), "task_manager_node started");
}

void TaskManagerNode::on_intent(k1muse_voice_msgs::msg::IntentLite::SharedPtr msg)
{
  // Only handle action intents; query, reminder, system are ignored
  if (msg->intent_name != "action") {
    return;
  }

  const auto & action = msg->action;

  if (action == "move" || action == "find" || action == "rotate" ||
      action == "lift" || action == "pick" || action == "place" ||
      action == "pick_place" || action == "grasp" || action == "release") {
    send_goal(*msg);
  } else if (action == "stop") {
    cancel_active_goal();
    RCLCPP_INFO(get_logger(), "Stop requested, canceled active task");
  } else {
    RCLCPP_WARN(get_logger(), "Unknown action command: %s", action.c_str());
  }
}

void TaskManagerNode::send_goal(const k1muse_voice_msgs::msg::IntentLite & intent)
{
  // Cancel any in-flight goal before sending a new one
  cancel_active_goal();

  // Non-blocking check for action server availability.
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), "ExecuteTask action server not available");
    return;
  }

  // Build goal from IntentLite
  auto goal = ExecuteTask::Goal();
  goal.trace_id = intent.trace_id;
  goal.request_id = intent.request_id;
  goal.task_id = k1muse_common::make_id("task");
  goal.epoch = intent.epoch;
  goal.task_type = intent.action;

  // Set target_class based on action type
  if (intent.action == "find" || intent.action == "pick" ||
      intent.action == "place" || intent.action == "pick_place" ||
      intent.action == "grasp" ||
      intent.action == "release") {
    goal.target_class = intent.target;
    goal.target_id = intent.value;
  } else if (intent.action == "move" || intent.action == "lift") {
    goal.target_class = intent.value;
  }
  // target_id empty, resolved by supervisor/vision

  // Set timeout
  goal.timeout.sec = default_timeout_ms_ / 1000;
  goal.timeout.nanosec = static_cast<uint32_t>((default_timeout_ms_ % 1000) * 1000000);

  RCLCPP_INFO(get_logger(), "Sending %s task: task_id=%s target_class=%s",
              intent.action.c_str(), goal.task_id.c_str(), goal.target_class.c_str());

  // Store tracking info for status publishing
  auto trace_id = intent.trace_id;
  auto request_id = intent.request_id;
  auto task_id = goal.task_id;
  auto epoch = intent.epoch;

  // Store in member variables for result_callback race-condition check.
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_task_id_ = task_id;
    active_trace_id_ = trace_id;
    active_epoch_ = epoch;
  }

  // Send goal with callbacks
  auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();

  // Store tracking info for callbacks (captured by value).
  auto active_task_id = goal.task_id;

  send_goal_options.goal_response_callback =
    [this, trace_id, request_id, task_id, epoch](GoalHandle::SharedPtr handle)
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      active_goal_ = handle;
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "Goal rejected by server");
        publish_status(trace_id, request_id, task_id, epoch,
                       k1muse_manager_msgs::msg::TaskStatus::STATE_FAILED,
                       "rejected", 0.0f, "Goal rejected by server");
      } else {
        // Publish STATE_ACTIVE only after goal is accepted.
        publish_status(trace_id, request_id, task_id, epoch,
                       k1muse_manager_msgs::msg::TaskStatus::STATE_ACTIVE,
                       "active", 0.0f, "");
      }
    };

  send_goal_options.result_callback =
    [this, active_task_id](const GoalHandle::WrappedResult & result)
    {
      auto result_trace_id = result.result->trace_id;
      auto result_request_id = result.result->request_id;
      auto result_task_id = result.result->task_id;
      auto result_epoch = result.result->epoch;

      uint8_t state;
      std::string state_name;

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          state = k1muse_manager_msgs::msg::TaskStatus::STATE_SUCCEEDED;
          state_name = "succeeded";
          break;
        case rclcpp_action::ResultCode::ABORTED:
          state = k1muse_manager_msgs::msg::TaskStatus::STATE_FAILED;
          state_name = "failed";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          state = k1muse_manager_msgs::msg::TaskStatus::STATE_CANCELLED;
          state_name = "cancelled";
          break;
        default:
          state = k1muse_manager_msgs::msg::TaskStatus::STATE_FAILED;
          state_name = "unknown";
          break;
      }

      publish_status(result_trace_id, result_request_id, result_task_id, result_epoch,
                     state, state_name,
                     (state == k1muse_manager_msgs::msg::TaskStatus::STATE_SUCCEEDED) ? 1.0f : 0.0f,
                     result.result->reason);

      // Only clear tracking if this result belongs to the currently tracked goal.
      {
        std::lock_guard<std::mutex> lock(active_mutex_);
        if (active_task_id_ == active_task_id) {
          active_goal_.reset();
          active_task_id_.clear();
          active_trace_id_.clear();
          active_epoch_ = 0;
        }
      }
    };

  action_client_->async_send_goal(goal, send_goal_options);
}

void TaskManagerNode::cancel_active_goal()
{
  std::lock_guard<std::mutex> lock(active_mutex_);
  if (active_goal_) {
    action_client_->async_cancel_goal(active_goal_);
  }
}

void TaskManagerNode::publish_status(
  const std::string & trace_id,
  const std::string & request_id,
  const std::string & task_id,
  uint64_t epoch,
  uint8_t state,
  const std::string & state_name,
  float progress,
  const std::string & reason)
{
  auto msg = k1muse_manager_msgs::msg::TaskStatus();
  msg.header.stamp = now();
  msg.trace_id = trace_id;
  msg.request_id = request_id;
  msg.task_id = task_id;
  msg.epoch = epoch;
  msg.state = state;
  msg.state_name = state_name;
  msg.progress = progress;
  msg.reason = reason;
  status_publisher_->publish(msg);
}

}  // namespace k1muse_task_manager
