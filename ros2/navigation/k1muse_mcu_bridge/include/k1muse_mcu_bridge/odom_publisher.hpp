#pragma once

#include <array>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "k1muse_mcu_bridge/msg/chassis_status.hpp"

namespace k1muse_mcu_bridge {

class OdomPublisher {
public:
  struct Params {
    double publish_rate_hz = 50.0;
    bool publish_tf = true;
    std::string odom_frame = "odom";
    std::string base_frame = "base_footprint";
    std::array<double, 6> pose_cov_diag = {0.02, 0.02, 0.01, 0.01, 0.01, 0.05};
    std::array<double, 6> twist_cov_diag = {0.05, 0.05, 0.01, 0.01, 0.01, 0.10};
  };

  explicit OdomPublisher(rclcpp::Node* node);

private:
  void status_callback(const msg::ChassisStatus::SharedPtr msg);

  rclcpp::Node* node_;
  Params params_;
  rclcpp::Subscription<msg::ChassisStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_publish_time_;
};

}  // namespace k1muse_mcu_bridge
