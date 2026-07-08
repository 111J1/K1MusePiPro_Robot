#include "k1muse_vision_3d/vision_3d_node.hpp"

#include <chrono>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include "k1muse_common/qos_profiles.hpp"
#include "k1muse_vision_3d/geometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace k1muse_vision_3d
{

Vision3DNode::Vision3DNode(const rclcpp::NodeOptions & options)
: Node("vision_3d_node", options)
{
  // Declare and read parameters.
  window_px_ = declare_parameter<int>("depth_roi_window_px", window_px_);
  min_depth_m_ = declare_parameter<double>("depth_min_m", min_depth_m_);
  max_depth_m_ = declare_parameter<double>("depth_max_m", max_depth_m_);
  frame_ttl_ms_ = declare_parameter<double>("frame_ttl_ms", frame_ttl_ms_);
  target_frame_ = declare_parameter<std::string>("target_frame", target_frame_);
  camera_frame_fallback_ =
    declare_parameter<std::string>("camera_frame_fallback", camera_frame_fallback_);
  tf_timeout_ms_ = declare_parameter<double>("tf_timeout_ms", tf_timeout_ms_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  detection_sub_ = create_subscription<Detection2DFrame>(
    "/vision/detection_2d", k1muse_common::qos::ReliableEvent(5),
    std::bind(&Vision3DNode::on_detection_2d, this, std::placeholders::_1));

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/main/depth_registered/image_raw", k1muse_common::qos::SensorLatest(3),
    std::bind(&Vision3DNode::on_depth_image, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/main/color/camera_info", k1muse_common::qos::SensorLatest(3),
    std::bind(&Vision3DNode::on_camera_info, this, std::placeholders::_1));

  target_response_sub_ = create_subscription<TargetResponse>(
    "/vision/target_response", k1muse_common::qos::ReliableEvent(5),
    std::bind(&Vision3DNode::on_target_response, this, std::placeholders::_1));

  // Publisher
  target_3d_publisher_ = create_publisher<Target3D>(
    "/vision/main/target_3d", k1muse_common::qos::ReliableResult(5));

  RCLCPP_INFO(get_logger(),
    "[startup] vision_3d window=%d depth=[%.1f,%.1f] ttl_ms=%.0f "
    "target_frame=%s camera_frame_fallback=%s tf_timeout_ms=%.0f "
    "topics={detection_in:/vision/detection_2d "
    "depth_in:/camera/main/depth_registered/image_raw "
    "camera_info_in:/camera/main/color/camera_info "
    "target_response_in:/vision/target_response "
    "target_3d_out:/vision/main/target_3d}",
    window_px_, min_depth_m_, max_depth_m_, frame_ttl_ms_,
    target_frame_.c_str(), camera_frame_fallback_.c_str(), tf_timeout_ms_);
}

void Vision3DNode::on_detection_2d(Detection2DFrame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  latest_detection_ = std::move(msg);
}

void Vision3DNode::on_depth_image(sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  latest_depth_ = std::move(msg);
}

void Vision3DNode::on_camera_info(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  latest_camera_info_ = std::move(msg);
}

void Vision3DNode::publish_invalid_target3d(
  const std::string & request_id,
  const std::string & trace_id,
  const std::string & target_id,
  uint64_t epoch,
  const std::string & reason)
{
  Target3D out;
  out.header.stamp = now();
  out.header.frame_id = target_frame_;
  out.request_id = request_id;
  out.trace_id = trace_id;
  out.epoch = epoch;
  out.target_id = target_id;
  out.valid = false;
  out.reason = reason;
  target_3d_publisher_->publish(out);
  RCLCPP_INFO(get_logger(),
    "[trace] target3d invalid trace_id=%s request_id=%s target_id=%s "
    "epoch=%llu reason=%s",
    trace_id.c_str(), request_id.c_str(), target_id.c_str(),
    static_cast<unsigned long long>(epoch), reason.c_str());
}

void Vision3DNode::on_target_response(TargetResponse::SharedPtr msg)
{
  // Only process found targets.
  if (!msg->found) {
    return;
  }

  Detection2DFrame::SharedPtr detection_frame;
  sensor_msgs::msg::Image::SharedPtr depth_image;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    detection_frame = latest_detection_;
    depth_image = latest_depth_;
    camera_info = latest_camera_info_;
  }

  // Check that we have a detection frame.
  if (!detection_frame) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "detection_not_found");
    return;
  }

  // Find the detection with matching target_id.
  const k1muse_vision_msgs::msg::Detection2D * matched = nullptr;
  for (const auto & det : detection_frame->detections) {
    if (det.detection_id == msg->target_id) {
      matched = &det;
      break;
    }
  }
  if (matched == nullptr) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "detection_not_found");
    return;
  }

  // Check frame TTL.
  const auto detection_stamp = rclcpp::Time(detection_frame->header.stamp);
  const auto now_time = now();
  const double age_ms = (now_time - detection_stamp).nanoseconds() / 1.0e6;
  if (age_ms > frame_ttl_ms_) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "frame_expired");
    return;
  }

  // Check depth and camera info availability.
  if (!depth_image) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "invalid_depth");
    return;
  }
  if (!camera_info) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "invalid_depth");
    return;
  }

  // Compute detection center.
  const double center_u = static_cast<double>(matched->x) +
    static_cast<double>(matched->width) / 2.0;
  const double center_v = static_cast<double>(matched->y) +
    static_cast<double>(matched->height) / 2.0;

  // Build depth image view.
  geometry::DepthImageView depth_view;
  depth_view.width = static_cast<int>(depth_image->width);
  depth_view.height = static_cast<int>(depth_image->height);
  depth_view.encoding = depth_image->encoding;
  depth_view.data = depth_image->data;

  // Compute median depth.
  const auto depth_result = geometry::MedianDepthMeters(
    depth_view, center_u, center_v, window_px_, min_depth_m_, max_depth_m_);
  if (!depth_result.valid) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "invalid_depth");
    return;
  }

  // Extract camera intrinsics from CameraInfo.k[] (3x3 matrix).
  geometry::CameraIntrinsics intrinsics;
  intrinsics.fx = camera_info->k[0];
  intrinsics.fy = camera_info->k[4];
  intrinsics.cx = camera_info->k[2];
  intrinsics.cy = camera_info->k[5];

  // Back-project to 3D.
  const auto point = geometry::BackProject(
    center_u, center_v, static_cast<double>(depth_result.depth_m), intrinsics);
  if (!point.valid) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "invalid_depth");
    return;
  }

  const std::string source_frame =
    !depth_image->header.frame_id.empty() ? depth_image->header.frame_id :
    !camera_info->header.frame_id.empty() ? camera_info->header.frame_id :
    camera_frame_fallback_;
  if (source_frame.empty() || target_frame_.empty()) {
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "missing_frame_id");
    return;
  }

  geometry_msgs::msg::PointStamped camera_point;
  camera_point.header.stamp = depth_image->header.stamp;
  camera_point.header.frame_id = source_frame;
  camera_point.point.x = point.x;
  camera_point.point.y = point.y;
  camera_point.point.z = point.z;

  geometry_msgs::msg::PointStamped target_point;
  try {
    if (source_frame == target_frame_) {
      target_point = camera_point;
    } else {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame_, source_frame, camera_point.header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_ms_ / 1000.0));
      tf2::doTransform(camera_point, target_point, transform);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(),
      "Failed to transform Target3D point %s -> %s: %s",
      source_frame.c_str(), target_frame_.c_str(), ex.what());
    publish_invalid_target3d(
      msg->request_id, msg->trace_id, msg->target_id, msg->epoch,
      "tf_unavailable");
    return;
  }

  // Publish valid Target3D.
  Target3D out;
  out.header.stamp = now();
  out.header.frame_id = target_frame_;
  out.request_id = msg->request_id;
  out.trace_id = msg->trace_id;
  out.epoch = msg->epoch;
  out.target_id = msg->target_id;
  out.x = static_cast<float>(target_point.point.x);
  out.y = static_cast<float>(target_point.point.y);
  out.z = static_cast<float>(target_point.point.z);
  out.depth = depth_result.depth_m;
  out.valid = true;
  out.reason = "ok";
  target_3d_publisher_->publish(out);
  RCLCPP_INFO(get_logger(),
    "[trace] target3d valid trace_id=%s request_id=%s target_id=%s "
    "epoch=%llu depth=%.3f xyz=(%.3f,%.3f,%.3f)",
    out.trace_id.c_str(), out.request_id.c_str(), out.target_id.c_str(),
    static_cast<unsigned long long>(out.epoch), out.depth, out.x, out.y, out.z);
}

}  // namespace k1muse_vision_3d
