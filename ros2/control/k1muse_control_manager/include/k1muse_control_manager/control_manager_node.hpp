#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "k1muse_manager_msgs/action/execute_task.hpp"
#include "k1muse_vision_msgs/msg/target3_d.hpp"
#include "k1muse_vision_msgs/msg/target_request.hpp"
#include "k1muse_vision_msgs/msg/target_response.hpp"

#include "k1muse_control_manager/device_session.hpp"
#include "k1muse_control_manager/execution_policy.hpp"
#include "k1muse_control_manager/grasp_profile_store.hpp"
#include "k1muse_control_manager/grasp_task_executor.hpp"
#include "k1muse_control_manager/mock_task_executor.hpp"
#include "k1muse_control_manager/stm32_actuator_client.hpp"

namespace k1muse_control_manager
{

using ExecuteTask = k1muse_manager_msgs::action::ExecuteTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

class ControlManagerNode : public rclcpp::Node
{
public:
  explicit ControlManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControlManagerNode();

private:
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTask::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle);

  void execute(
    const std::shared_ptr<GoalHandle> goal_handle);

  void cleanup_task_threads();

  // Find flow
  void execute_find(
    const std::shared_ptr<GoalHandle> goal_handle);
  TaskResult execute_pick_goal(
    const std::shared_ptr<const ExecuteTask::Goal> & goal);
  TaskResult execute_place_goal(
    const std::shared_ptr<const ExecuteTask::Goal> & goal);
  TaskResult execute_pick_place_goal(
    const std::shared_ptr<const ExecuteTask::Goal> & goal);
  TaskResult execute_release_goal(
    const std::shared_ptr<const ExecuteTask::Goal> & goal);
  void on_target_response(k1muse_vision_msgs::msg::TargetResponse::SharedPtr msg);
  void on_target_3d(k1muse_vision_msgs::msg::Target3D::SharedPtr msg);

  builtin_interfaces::msg::Time now_ros() const;
  int get_timeout_ms(const std::shared_ptr<const ExecuteTask::Goal> & goal) const;
  void configure_backend();
  ExecutionPolicy read_execution_policy();
  std::string resolve_profile_path(const std::string & configured_path) const;
  void publish_lift_joint_state();

  // Action server
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  MockTaskExecutor executor_;

  // Find flow synchronization
  std::mutex find_mutex_;
  std::condition_variable find_cv_;
  k1muse_vision_msgs::msg::TargetResponse latest_target_response_;
  bool target_response_received_{false};
  k1muse_vision_msgs::msg::Target3D latest_target_3d_;
  bool target_3d_received_{false};

  // Publishers/subscribers for find flow
  rclcpp::Publisher<k1muse_vision_msgs::msg::TargetRequest>::SharedPtr
    target_request_publisher_;
  rclcpp::Subscription<k1muse_vision_msgs::msg::TargetResponse>::SharedPtr
    target_response_subscription_;
  rclcpp::Subscription<k1muse_vision_msgs::msg::Target3D>::SharedPtr
    target_3d_subscription_;

  // Configuration
  int mock_delay_ms_{500};
  int find_timeout_ms_{5000};
  std::string backend_{"mock"};
  bool stop_all_on_cancel_{true};
  bool stop_all_on_exception_{true};
  bool publish_lift_joint_state_{true};
  int lift_joint_state_period_ms_{50};
  std::string lift_joint_name_{"lift_joint"};

  // Coordinate-state publication for robot_state_publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;

  // Real STM32 backend. These are only constructed when backend=stm32_uart.
  GraspProfileStore grasp_profiles_;
  std::unique_ptr<Stm32ActuatorClient> actuator_client_;
  std::unique_ptr<DeviceSession> device_session_;
  std::unique_ptr<GraspTaskExecutor> grasp_executor_;

  // Task thread management
  std::mutex task_threads_mutex_;
  std::vector<std::thread> task_threads_;
};

}  // namespace k1muse_control_manager
