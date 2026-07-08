#include "k1muse_control_manager/control_manager_node.hpp"

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "k1muse_common/id_utils.hpp"
#include "k1muse_common/qos_profiles.hpp"

namespace k1muse_control_manager
{

ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions & options)
: Node("control_manager_node", options)
{
  mock_delay_ms_ = declare_parameter<int>("mock_delay_ms", 500);
  find_timeout_ms_ = declare_parameter<int>("find_timeout_ms", 5000);
  backend_ = declare_parameter<std::string>("backend", "mock");
  stop_all_on_cancel_ = declare_parameter<bool>("safety.stop_all_on_cancel", true);
  stop_all_on_exception_ = declare_parameter<bool>("safety.stop_all_on_exception", true);
  publish_lift_joint_state_ =
    declare_parameter<bool>("tf.publish_lift_joint_state", true);
  lift_joint_state_period_ms_ =
    declare_parameter<int>("tf.lift_joint_state_period_ms", 50);
  lift_joint_name_ =
    declare_parameter<std::string>("tf.lift_joint_name", "lift_joint");
  if (lift_joint_state_period_ms_ <= 0) {
    RCLCPP_WARN(get_logger(),
      "Invalid tf.lift_joint_state_period_ms=%d, using 50ms",
      lift_joint_state_period_ms_);
    lift_joint_state_period_ms_ = 50;
  }

  configure_backend();

  if (backend_ == "stm32_uart" && publish_lift_joint_state_) {
    joint_state_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    joint_state_timer_ = create_wall_timer(
      std::chrono::milliseconds(lift_joint_state_period_ms_),
      std::bind(&ControlManagerNode::publish_lift_joint_state, this));
    RCLCPP_INFO(get_logger(),
      "Publishing lift joint state joint=%s topic=/joint_states period_ms=%d",
      lift_joint_name_.c_str(), lift_joint_state_period_ms_);
  }

  // Publishers
  target_request_publisher_ =
    create_publisher<k1muse_vision_msgs::msg::TargetRequest>(
      "/vision/target_request",
      k1muse_common::qos::ReliableEvent(5));

  // Subscribers
  target_response_subscription_ =
    create_subscription<k1muse_vision_msgs::msg::TargetResponse>(
      "/vision/target_response",
      k1muse_common::qos::ReliableEvent(5),
      [this](k1muse_vision_msgs::msg::TargetResponse::SharedPtr msg) {
        on_target_response(msg);
      });

  target_3d_subscription_ =
    create_subscription<k1muse_vision_msgs::msg::Target3D>(
      "/vision/main/target_3d",
      k1muse_common::qos::ReliableResult(5),
      [this](k1muse_vision_msgs::msg::Target3D::SharedPtr msg) {
        on_target_3d(msg);
      });

  // Action server
  action_server_ = rclcpp_action::create_server<ExecuteTask>(
    this,
    "execute_task",
    [this](const rclcpp_action::GoalUUID & uuid,
           std::shared_ptr<const ExecuteTask::Goal> goal)
        -> rclcpp_action::GoalResponse {
      return handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandle> goal_handle)
        -> rclcpp_action::CancelResponse {
      return handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandle> goal_handle) {
      handle_accepted(goal_handle);
    });

  RCLCPP_INFO(get_logger(), "control_manager_node started");
}

rclcpp_action::GoalResponse ControlManagerNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExecuteTask::Goal> goal)
{
  const auto & task_type = goal->task_type;
  if (task_type == "move" || task_type == "stop" ||
      task_type == "lift" || task_type == "rotate" ||
      task_type == "find" || task_type == "pick" ||
      task_type == "place" || task_type == "pick_place" ||
      task_type == "grasp" ||
      task_type == "release") {
    RCLCPP_INFO(get_logger(), "Accepting goal with task_type: %s", task_type.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  RCLCPP_WARN(get_logger(), "Rejecting unknown task_type: %s", task_type.c_str());
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ControlManagerNode::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Cancel requested");
  if (stop_all_on_cancel_ && actuator_client_) {
    actuator_client_->stop_all();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ControlManagerNode::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(task_threads_mutex_);
  task_threads_.emplace_back([this, goal_handle]() { execute(goal_handle); });
}

ControlManagerNode::~ControlManagerNode()
{
  cleanup_task_threads();
}

void ControlManagerNode::cleanup_task_threads()
{
  std::lock_guard<std::mutex> lock(task_threads_mutex_);
  for (auto& t : task_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  task_threads_.clear();
}

void ControlManagerNode::execute(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto & task_type = goal->task_type;
  int duration_ms = mock_delay_ms_;
  get_parameter("mock_delay_ms", duration_ms);

  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  auto result = std::make_shared<ExecuteTask::Result>();
  result->header.stamp = now_ros();
  result->trace_id = goal->trace_id;
  result->request_id = goal->request_id;
  result->task_id = goal->task_id;
  result->epoch = goal->epoch;
  feedback->header.stamp = now_ros();
  feedback->trace_id = goal->trace_id;
  feedback->request_id = goal->request_id;
  feedback->task_id = goal->task_id;
  feedback->epoch = goal->epoch;

  if (task_type == "find") {
    execute_find(goal_handle);
    return;
  }

  // Mock or real execution depending on backend and task type.
  feedback->state = 1;
  feedback->state_name = "active";
  feedback->progress = 0.5f;
  feedback->detail = "Executing " + task_type;
  goal_handle->publish_feedback(feedback);

  TaskResult task_result;
  if (task_type == "move") {
    if (backend_ == "stm32_uart") {
      task_result = make_failure("STM32 move task is not implemented yet");
    } else {
      task_result = executor_.execute_move(goal->target_class, duration_ms);
    }
  } else if (task_type == "stop") {
    if (backend_ == "stm32_uart" && actuator_client_) {
      task_result = actuator_client_->stop_all();
    } else {
      task_result = executor_.execute_stop(duration_ms);
    }
  } else if (task_type == "lift") {
    if (backend_ == "stm32_uart" && actuator_client_ && device_session_) {
      try {
        const float z_m = std::stof(goal->target_class);
        task_result = device_session_->ensure_lift_ready();
        if (task_result.success) {
          task_result = actuator_client_->lift_move_z(z_m);
        }
      } catch (const std::exception & e) {
        task_result = make_failure(std::string("invalid lift target: ") + e.what());
      }
    } else {
      task_result = executor_.execute_lift(goal->target_class, duration_ms);
    }
  } else if (task_type == "rotate") {
    if (backend_ == "stm32_uart") {
      task_result = make_failure("STM32 rotate task is not implemented yet");
    } else {
      task_result = executor_.execute_rotate(goal->target_class, duration_ms);
    }
  } else if (task_type == "pick" || task_type == "grasp") {
    task_result = execute_pick_goal(goal);
  } else if (task_type == "place") {
    task_result = execute_place_goal(goal);
  } else if (task_type == "pick_place") {
    task_result = execute_pick_place_goal(goal);
  } else if (task_type == "release") {
    task_result = execute_release_goal(goal);
  } else {
    result->success = false;
    result->final_state = 3;  // STATE_FAILED
    result->reason = "Unknown task_type: " + task_type;
    goal_handle->abort(result);
    return;
  }

  // Check for cancellation
  if (goal_handle->is_canceling()) {
    result->success = false;
    result->final_state = 4;  // STATE_CANCELLED
    result->reason = "Task cancelled";
    goal_handle->canceled(result);
    return;
  }

  if (task_result.success) {
    feedback->progress = 1.0f;
    feedback->state = 2;  // STATE_SUCCEEDED
    feedback->state_name = "succeeded";
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->final_state = 2;  // STATE_SUCCEEDED
    result->reason = task_result.reason;
    goal_handle->succeed(result);
  } else {
    result->success = false;
    result->final_state = 3;  // STATE_FAILED
    result->reason = task_result.reason;
    goal_handle->abort(result);
  }
}

TaskResult ControlManagerNode::execute_pick_goal(
  const std::shared_ptr<const ExecuteTask::Goal> & goal)
{
  if (backend_ != "stm32_uart") {
    return make_success("Mock pick completed");
  }
  if (!grasp_executor_) {
    return make_failure("STM32 pick executor is not configured");
  }
  if (goal->target_class.empty()) {
    return make_failure("pick target_class is required");
  }
  if (goal->target_z <= 0.0f) {
    return make_failure("pick target_z must be greater than zero");
  }

  try {
    const auto selection = grasp_profiles_.resolve(goal->target_class, goal->target_id);
    GraspTaskExecutor::Request request;
    request.geometry = selection.geometry;
    request.profile = selection.profile;
    request.target_z = goal->target_z;
    request.carry_pose = selection.carry_pose;
    return grasp_executor_->execute_pick(request);
  } catch (const std::exception & e) {
    if (stop_all_on_exception_ && actuator_client_) {
      actuator_client_->stop_all();
    }
    return make_failure(e.what());
  }
}

TaskResult ControlManagerNode::execute_place_goal(
  const std::shared_ptr<const ExecuteTask::Goal> & goal)
{
  if (backend_ != "stm32_uart") {
    return make_success("Mock place completed");
  }
  if (!grasp_executor_) {
    return make_failure("STM32 place executor is not configured");
  }
  if (goal->target_class.empty()) {
    return make_failure("place target_class is required");
  }
  if (goal->target_z <= 0.0f) {
    return make_failure("place target_z must be greater than zero");
  }

  try {
    const auto selection = grasp_profiles_.resolve(goal->target_class, goal->target_id);
    GraspTaskExecutor::Request request;
    request.geometry = selection.geometry;
    request.profile = selection.profile;
    request.target_z = goal->target_z;
    request.carry_pose = selection.carry_pose;
    return grasp_executor_->execute_place(request);
  } catch (const std::exception & e) {
    if (stop_all_on_exception_ && actuator_client_) {
      actuator_client_->stop_all();
    }
    return make_failure(e.what());
  }
}

TaskResult ControlManagerNode::execute_pick_place_goal(
  const std::shared_ptr<const ExecuteTask::Goal> & goal)
{
  if (backend_ != "stm32_uart") {
    return make_success("Mock pick_place completed");
  }
  if (goal->place_z <= 0.0f) {
    return make_failure("pick_place place_z must be greater than zero");
  }

  auto pick_result = execute_pick_goal(goal);
  if (!pick_result.success) {
    return pick_result;
  }

  auto place_goal = std::make_shared<ExecuteTask::Goal>(*goal);
  place_goal->target_z = goal->place_z;
  return execute_place_goal(place_goal);
}

TaskResult ControlManagerNode::execute_release_goal(
  const std::shared_ptr<const ExecuteTask::Goal> & goal)
{
  if (backend_ != "stm32_uart") {
    return make_success("Mock release completed");
  }
  if (!grasp_executor_) {
    return make_failure("STM32 grasp executor is not configured");
  }

  float open_angle = grasp_profiles_.default_open_angle_rad();
  try {
    if (!goal->target_class.empty()) {
      open_angle = grasp_profiles_.resolve(
        goal->target_class, goal->target_id).profile.gripper_open_rad;
    }
    return grasp_executor_->release(open_angle);
  } catch (const std::exception & e) {
    if (stop_all_on_exception_ && actuator_client_) {
      actuator_client_->stop_all();
    }
    return make_failure(e.what());
  }
}

void ControlManagerNode::execute_find(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ExecuteTask::Result>();
  result->header.stamp = now_ros();
  result->trace_id = goal->trace_id;
  result->request_id = goal->request_id;
  result->task_id = goal->task_id;
  result->epoch = goal->epoch;
  const int timeout_ms = get_timeout_ms(goal);
  const std::string request_id = k1muse_common::make_id("req");

  // Step 1: Publish target request
  k1muse_vision_msgs::msg::TargetRequest request;
  request.header.stamp = now_ros();
  request.request_id = request_id;
  request.trace_id = goal->trace_id;
  request.epoch = goal->epoch;
  request.target_class = goal->target_class;
  request.minimum_score = 0.5f;
  request.timeout_ms = static_cast<uint32_t>(timeout_ms);
  target_request_publisher_->publish(request);

  RCLCPP_INFO(get_logger(), "Published target request for '%s', request_id=%s",
    goal->target_class.c_str(), request_id.c_str());

  // Step 2: Wait for target response
  {
    std::unique_lock<std::mutex> lock(find_mutex_);
    target_response_received_ = false;
    target_3d_received_ = false;
  }

  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);

  {
    std::unique_lock<std::mutex> lock(find_mutex_);
    bool received = find_cv_.wait_until(lock, deadline, [this, request_id]() {
      return target_response_received_ &&
        latest_target_response_.request_id == request_id;
    });

    if (!received) {
      result->success = false;
      result->final_state = 3;  // STATE_FAILED
      result->reason = "Timeout waiting for target response";
      goal_handle->abort(result);
      return;
    }

    if (!latest_target_response_.found) {
      result->success = false;
      result->final_state = 3;  // STATE_FAILED
      result->reason = "Target not found: " + latest_target_response_.reason;
      goal_handle->abort(result);
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "Target found, target_id=%s. Waiting for 3D position...",
    latest_target_response_.target_id.c_str());

  // Step 3: Wait for target 3D with matching request_id
  {
    std::unique_lock<std::mutex> lock(find_mutex_);
    bool received = find_cv_.wait_until(lock, deadline, [this, request_id]() {
      return target_3d_received_ &&
        latest_target_3d_.request_id == request_id;
    });

    if (!received) {
      result->success = false;
      result->final_state = 3;  // STATE_FAILED
      result->reason = "Timeout waiting for target 3D position";
      goal_handle->abort(result);
      return;
    }

    if (!latest_target_3d_.valid) {
      result->success = false;
      result->final_state = 3;  // STATE_FAILED
      result->reason = "Target 3D invalid: " + latest_target_3d_.reason;
      goal_handle->abort(result);
      return;
    }
  }

  // Step 4: Return success with coordinates
  result->success = true;
  result->final_state = 2;  // STATE_SUCCEEDED
  result->reason = "Target found at (" +
    std::to_string(latest_target_3d_.x) + ", " +
    std::to_string(latest_target_3d_.y) + ", " +
    std::to_string(latest_target_3d_.z) + ")";
  goal_handle->succeed(result);
}

void ControlManagerNode::on_target_response(
  k1muse_vision_msgs::msg::TargetResponse::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(find_mutex_);
  latest_target_response_ = *msg;
  target_response_received_ = true;
  find_cv_.notify_all();
}

void ControlManagerNode::on_target_3d(
  k1muse_vision_msgs::msg::Target3D::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(find_mutex_);
  latest_target_3d_ = *msg;
  target_3d_received_ = true;
  find_cv_.notify_all();
}

void ControlManagerNode::publish_lift_joint_state()
{
  if (!joint_state_publisher_ || !actuator_client_) {
    return;
  }

  const auto status = actuator_client_->latest_lift_status();
  if (!status) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name.push_back(lift_joint_name_);
  msg.position.push_back(status->current_z);
  joint_state_publisher_->publish(msg);
}

builtin_interfaces::msg::Time ControlManagerNode::now_ros() const
{
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

int ControlManagerNode::get_timeout_ms(
  const std::shared_ptr<const ExecuteTask::Goal> & goal) const
{
  const auto & timeout = goal->timeout;
  int timeout_ms = static_cast<int>(timeout.sec) * 1000 +
    static_cast<int>(timeout.nanosec) / 1000000;
  if (timeout_ms <= 0) {
    timeout_ms = find_timeout_ms_;
  }
  return timeout_ms;
}

void ControlManagerNode::configure_backend()
{
  if (backend_ == "mock") {
    RCLCPP_INFO(get_logger(), "control backend: mock");
    return;
  }
  if (backend_ != "stm32_uart") {
    throw std::runtime_error("unsupported control backend: " + backend_);
  }

  const std::string port = declare_parameter<std::string>("serial.port", "/dev/ttyS2");
  const int baud_rate = declare_parameter<int>("serial.baud_rate", 115200);
  const std::string source_name = declare_parameter<std::string>("serial.source", "host");
  const int read_timeout_ms = declare_parameter<int>("serial.read_timeout_ms", 20);
  const std::string profile_path_config =
    declare_parameter<std::string>("profiles.grasp_profiles", "grasp_profiles.yaml");

  uint8_t source = static_cast<uint8_t>(K1_CTRL_SRC_HOST);
  if (source_name == "bt") {
    source = static_cast<uint8_t>(K1_CTRL_SRC_BT);
  } else if (source_name != "host") {
    throw std::runtime_error("unsupported serial.source: " + source_name);
  }

  grasp_profiles_.load_from_file(resolve_profile_path(profile_path_config));

  Stm32ActuatorClient::Options options;
  options.source = source;
  options.policy = read_execution_policy();
  options.read_timeout = std::chrono::milliseconds(read_timeout_ms);

  auto transport = std::make_unique<SerialTransport>(port, baud_rate);
  actuator_client_ = std::make_unique<Stm32ActuatorClient>(std::move(transport), options);
  actuator_client_->start();
  device_session_ = std::make_unique<DeviceSession>(*actuator_client_);
  grasp_executor_ = std::make_unique<GraspTaskExecutor>(*actuator_client_, *device_session_);

  RCLCPP_INFO(
    get_logger(), "control backend: stm32_uart port=%s baud=%d source=%s",
    port.c_str(), baud_rate, source_name.c_str());
}

ExecutionPolicy ControlManagerNode::read_execution_policy()
{
  ExecutionPolicy policy;
  policy.mode = parse_execution_policy_mode(
    declare_parameter<std::string>("execution_policy", "demo_tolerant"));
  policy.result_timeout = std::chrono::milliseconds(
    declare_parameter<int>("serial.result_timeout_ms", 15000));
  policy.gripper_timeout = std::chrono::milliseconds(
    declare_parameter<int>("gripper.result_timeout_ms", 1000));
  policy.arm_pose_xyz_tolerance_m =
    declare_parameter<double>("tolerances.arm_pose_xyz_tolerance_m", 0.015);
  policy.lift_z_tolerance_m =
    declare_parameter<double>("tolerances.lift_z_tolerance_m", 0.02);
  policy.gripper_tolerance_rad =
    declare_parameter<double>("tolerances.gripper_tolerance_rad", 0.05);
  policy.assume_gripper_success_without_fault =
    declare_parameter<bool>("gripper.assume_success_without_fault", true);
  policy.rehome_lift_on_fault =
    declare_parameter<bool>("readiness.lift_rehome_on_fault", true);
  return policy;
}

std::string ControlManagerNode::resolve_profile_path(
  const std::string & configured_path) const
{
  namespace fs = std::filesystem;

  fs::path path(configured_path);
  if (path.is_absolute() && fs::exists(path)) {
    return path.string();
  }

  try {
    const fs::path share =
      ament_index_cpp::get_package_share_directory("k1muse_control_manager");
    const fs::path candidate = share / "config" / path;
    if (fs::exists(candidate)) {
      return candidate.string();
    }
  } catch (const std::exception &) {
  }

  const fs::path cwd_candidate = fs::current_path() / path;
  if (fs::exists(cwd_candidate)) {
    return cwd_candidate.string();
  }

  const fs::path source_candidate =
    fs::current_path() / "k1muse_control_manager" / "config" / path;
  if (fs::exists(source_candidate)) {
    return source_candidate.string();
  }

  throw std::runtime_error("grasp profile file not found: " + configured_path);
}

}  // namespace k1muse_control_manager
