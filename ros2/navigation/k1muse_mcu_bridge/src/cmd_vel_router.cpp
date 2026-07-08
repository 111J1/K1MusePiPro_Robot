#include "k1muse_mcu_bridge/cmd_vel_router.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace k1muse_mcu_bridge {
namespace {

constexpr const char * kDebugLogDir =
    "/home/bianbu/k1muse_communicate_ros/src/k1muse_mobile_bridge/debug_logs";

int64_t wall_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

void append_debug_line(const std::string& event, const std::string& fields) {
  static std::mutex log_mutex;
  std::lock_guard<std::mutex> lock(log_mutex);
  std::filesystem::create_directories(kDebugLogDir);
  std::ofstream out(std::filesystem::path(kDebugLogDir) / "mcu_bridge_debug.tsv",
                    std::ios::app);
  out << wall_ms() << '\t' << event << '\t' << fields << '\n';
}

}  // namespace

CmdVelRouter::CmdVelRouter(rclcpp::Node* node)
    : node_(node) {
  node_->declare_parameter("vx_limit", 1.4);
  node_->declare_parameter("vy_limit", 1.4);
  node_->declare_parameter("omega_limit", 3.7);
  node_->declare_parameter("cmd_vel_timeout_ms", 250);
  node_->declare_parameter("cmd_vel_rate_hz", 20);
  node_->declare_parameter("zero_vel_send_stop", true);
  node_->declare_parameter("enable_cmd_vel_output", false);

  node_->get_parameter("vx_limit", params_.vx_limit);
  node_->get_parameter("vy_limit", params_.vy_limit);
  node_->get_parameter("omega_limit", params_.omega_limit);
  node_->get_parameter("cmd_vel_timeout_ms", params_.cmd_vel_timeout_ms);
  node_->get_parameter("cmd_vel_rate_hz", params_.cmd_vel_rate_hz);
  node_->get_parameter("zero_vel_send_stop", params_.zero_vel_send_stop);
  node_->get_parameter("enable_cmd_vel_output", params_.enable_cmd_vel_output);

  if (!params_.enable_cmd_vel_output) {
    RCLCPP_INFO(
        node_->get_logger(),
        "CmdVelRouter disabled: read-only odom mode, no /cmd_vel to MCU output");
    return;
  }

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelRouter::cmd_vel_callback, this, std::placeholders::_1));

  mov_pub_ = node_->create_publisher<msg::ChassisMov>("/mcu/chassis/mov", 10);
  stop_client_ = node_->create_client<srv::ChassisStop>("/mcu/chassis/stop");

  last_cmd_vel_time_ = node_->now();

  if (params_.cmd_vel_rate_hz <= 0) {
    RCLCPP_WARN(node_->get_logger(), "cmd_vel_rate_hz=%d is invalid, using 20 Hz",
                params_.cmd_vel_rate_hz);
    params_.cmd_vel_rate_hz = 20;
  }

  const auto period = std::chrono::milliseconds(1000 / params_.cmd_vel_rate_hz);
  timer_ = node_->create_wall_timer(period, std::bind(&CmdVelRouter::timer_callback, this));

  RCLCPP_INFO(
      node_->get_logger(),
      "CmdVelRouter ready (limits: |vx|<=%.2f |vy|<=%.2f |omega|<=%.2f, "
      "timeout=%dms, rate=%dHz, zero_stop=%d, output=%d)",
      params_.vx_limit, params_.vy_limit, params_.omega_limit,
      params_.cmd_vel_timeout_ms, params_.cmd_vel_rate_hz, params_.zero_vel_send_stop,
      params_.enable_cmd_vel_output);
}

void CmdVelRouter::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  const double vx = std::clamp(msg->linear.x, -params_.vx_limit, params_.vx_limit);
  const double vy = std::clamp(msg->linear.y, -params_.vy_limit, params_.vy_limit);
  const double omega =
      std::clamp(msg->angular.z, -params_.omega_limit, params_.omega_limit);
  append_debug_line(
      "cmd_vel_rx",
      "vx=" + std::to_string(msg->linear.x) +
          "\tvy=" + std::to_string(msg->linear.y) +
          "\twz=" + std::to_string(msg->angular.z) +
          "\tclamped_vx=" + std::to_string(vx) +
          "\tclamped_vy=" + std::to_string(vy) +
          "\tclamped_wz=" + std::to_string(omega));

  static constexpr double EPS = 1e-6;
  if (params_.zero_vel_send_stop && std::abs(vx) < EPS && std::abs(vy) < EPS &&
      std::abs(omega) < EPS) {
    send_stop();
    return;
  }

  const double v = std::hypot(vx, vy);
  const double direction = std::atan2(vy, vx);

  send_mov(static_cast<float>(direction), static_cast<float>(v), static_cast<float>(omega));
  last_cmd_vel_time_ = node_->now();
}

void CmdVelRouter::timer_callback() {
  const auto now = node_->now();
  const int elapsed_ms = static_cast<int>((now - last_cmd_vel_time_).seconds() * 1000);

  if (elapsed_ms > params_.cmd_vel_timeout_ms) {
    if (has_last_cmd_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "cmd_vel timeout (%d ms > %d ms), sending STOP", elapsed_ms,
          params_.cmd_vel_timeout_ms);
      has_last_cmd_ = false;
      send_stop();
    }
    return;
  }

  if (has_last_cmd_) {
    mov_pub_->publish(last_mov_cmd_);
  }
}

void CmdVelRouter::send_mov(float direction, float v, float omega) {
  last_mov_cmd_.move_cs = 0;
  last_mov_cmd_.direction = direction;
  last_mov_cmd_.v = v;
  last_mov_cmd_.omega = omega;
  has_last_cmd_ = true;
  stop_sent_ = false;
  mov_pub_->publish(last_mov_cmd_);
  append_debug_line(
      "mov_publish",
      "direction=" + std::to_string(direction) +
          "\tv=" + std::to_string(v) +
          "\tomega=" + std::to_string(omega));
}

void CmdVelRouter::send_stop() {
  has_last_cmd_ = false;
  if (stop_sent_) {
    append_debug_line("stop_skip", "reason=duplicate_zero");
    return;
  }
  append_debug_line("stop_request", "service_ready=" +
                                        std::to_string(stop_client_->service_is_ready() ? 1 : 0));
  if (!stop_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "/mcu/chassis/stop service not available, cannot send STOP");
    return;
  }
  stop_sent_ = true;
  auto req = std::make_shared<srv::ChassisStop::Request>();
  stop_client_->async_send_request(req);
}

}  // namespace k1muse_mcu_bridge
