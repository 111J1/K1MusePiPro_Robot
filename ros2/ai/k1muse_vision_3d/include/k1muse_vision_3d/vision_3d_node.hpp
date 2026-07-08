#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "k1muse_vision_msgs/msg/detection2_d_frame.hpp"
#include "k1muse_vision_msgs/msg/target3_d.hpp"
#include "k1muse_vision_msgs/msg/target_response.hpp"

namespace k1muse_vision_3d
{

class Vision3DNode : public rclcpp::Node
{
public:
  using Detection2DFrame = k1muse_vision_msgs::msg::Detection2DFrame;
  using Target3D = k1muse_vision_msgs::msg::Target3D;
  using TargetResponse = k1muse_vision_msgs::msg::TargetResponse;

  explicit Vision3DNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Test accessors.
  void on_target_response(TargetResponse::SharedPtr msg);

private:
  void on_detection_2d(Detection2DFrame::SharedPtr msg);
  void on_depth_image(sensor_msgs::msg::Image::SharedPtr msg);
  void on_camera_info(sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void publish_invalid_target3d(
    const std::string & request_id,
    const std::string & trace_id,
    const std::string & target_id,
    uint64_t epoch,
    const std::string & reason);

  // Subscriptions
  rclcpp::Subscription<Detection2DFrame>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<TargetResponse>::SharedPtr target_response_sub_;

  // Publisher
  rclcpp::Publisher<Target3D>::SharedPtr target_3d_publisher_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Cached data (mutex-protected)
  std::mutex cache_mutex_;
  Detection2DFrame::SharedPtr latest_detection_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;

  // Config
  int window_px_ = 5;
  double min_depth_m_ = 0.1;
  double max_depth_m_ = 10.0;
  double frame_ttl_ms_ = 500.0;
  double tf_timeout_ms_ = 100.0;
  std::string target_frame_ = "base_link";
  std::string camera_frame_fallback_ = "depth_camera_optical_frame";
};

}  // namespace k1muse_vision_3d
