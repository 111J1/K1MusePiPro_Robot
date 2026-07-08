#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "k1muse_mcu_bridge/msg/chassis_mov.hpp"
#include "k1muse_mcu_bridge/srv/chassis_stop.hpp"

namespace k1muse_mcu_bridge {

class CmdVelRouter {
public:
  struct Params {
    double vx_limit = 1.4;
    double vy_limit = 1.4;
    double omega_limit = 3.7;
    int cmd_vel_timeout_ms = 250;
    int cmd_vel_rate_hz = 20;
    bool zero_vel_send_stop = true;
    bool enable_cmd_vel_output = false;
  };

  explicit CmdVelRouter(rclcpp::Node* node);

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback();
  void send_mov(float direction, float v, float omega);
  void send_stop();

  rclcpp::Node* node_;
  Params params_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<msg::ChassisMov>::SharedPtr mov_pub_;
  rclcpp::Client<srv::ChassisStop>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_cmd_vel_time_;
  msg::ChassisMov last_mov_cmd_;
  bool has_last_cmd_ = false;
  bool stop_sent_ = false;
};

}  // namespace k1muse_mcu_bridge
