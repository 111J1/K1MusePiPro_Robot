#include "k1muse_mcu_bridge/odom_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace k1muse_mcu_bridge {

OdomPublisher::OdomPublisher(rclcpp::Node* node)
    : node_(node) {
  node_->declare_parameter("odom_frame", "odom");
  node_->declare_parameter("base_frame", "base_footprint");
  node_->declare_parameter("publish_tf", true);
  node_->declare_parameter("odom_publish_rate_hz", 50.0);
  node_->declare_parameter(
      "pose_covariance_diag",
      std::vector<double>{0.02, 0.02, 0.01, 0.01, 0.01, 0.05});
  node_->declare_parameter(
      "twist_covariance_diag",
      std::vector<double>{0.05, 0.05, 0.01, 0.01, 0.01, 0.10});

  node_->get_parameter("odom_frame", params_.odom_frame);
  node_->get_parameter("base_frame", params_.base_frame);
  node_->get_parameter("publish_tf", params_.publish_tf);
  node_->get_parameter("odom_publish_rate_hz", params_.publish_rate_hz);
  const auto pose_cov_vec = node_->get_parameter("pose_covariance_diag").as_double_array();
  const auto twist_cov_vec = node_->get_parameter("twist_covariance_diag").as_double_array();
  for (int i = 0; i < 6; ++i) {
    params_.pose_cov_diag[i] = pose_cov_vec[i];
    params_.twist_cov_diag[i] = twist_cov_vec[i];
  }

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  status_sub_ = node_->create_subscription<msg::ChassisStatus>(
      "/mcu/chassis/status", 10,
      std::bind(&OdomPublisher::status_callback, this, std::placeholders::_1));

  last_publish_time_ = node_->now();

  RCLCPP_INFO(
      node_->get_logger(),
      "OdomPublisher ready (%.1f Hz, frame=%s -> %s, publish_tf=%d)",
      params_.publish_rate_hz, params_.odom_frame.c_str(), params_.base_frame.c_str(),
      params_.publish_tf);
}

void OdomPublisher::status_callback(const msg::ChassisStatus::SharedPtr msg) {
  const auto now = node_->now();
  const double period = 1.0 / params_.publish_rate_hz;
  if ((now - last_publish_time_).seconds() < period) {
    return;
  }
  last_publish_time_ = now;

  const double direction = msg->wcs_direction;
  const double cos_yaw = std::cos(direction);
  const double sin_yaw = std::sin(direction);

  tf2::Quaternion q;
  q.setRPY(0, 0, direction);

  const double base_vx = msg->wcs_vx * cos_yaw + msg->wcs_vy * sin_yaw;
  const double base_vy = -msg->wcs_vx * sin_yaw + msg->wcs_vy * cos_yaw;

  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = params_.odom_frame;
  odom_msg.child_frame_id = params_.base_frame;

  odom_msg.pose.pose.position.x = msg->wcs_x;
  odom_msg.pose.pose.position.y = msg->wcs_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.w = q.w();
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();

  odom_msg.twist.twist.linear.x = base_vx;
  odom_msg.twist.twist.linear.y = base_vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = msg->omega;

  for (int i = 0; i < 6; ++i) {
    odom_msg.pose.covariance[i * 6 + i] = params_.pose_cov_diag[i];
    odom_msg.twist.covariance[i * 6 + i] = params_.twist_cov_diag[i];
  }

  odom_pub_->publish(odom_msg);

  if (params_.publish_tf) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = params_.odom_frame;
    tf.child_frame_id = params_.base_frame;
    tf.transform.translation.x = msg->wcs_x;
    tf.transform.translation.y = msg->wcs_y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(tf);
  }
}

}  // namespace k1muse_mcu_bridge
