#include "k1muse_ai_runtime/alert_ros_publisher.hpp"

#include <sstream>

#include "k1muse_common/qos_profiles.hpp"

namespace k1muse_ai_runtime
{

AlertRosPublisher::AlertRosPublisher(rclcpp::Node* node)
    : node_(node)
{
  pub_ = node_->create_publisher<AiRuntimeAlert>(
      "/ai_runtime/alert",
      k1muse_common::qos::ReliableEvent(5));
}

void AlertRosPublisher::publish(const AlertEventPublisher::AlertInfo& info)
{
  if (!info.fired) return;

  AiRuntimeAlert msg;
  msg.header.stamp = node_->now();

  // Map alert type to uint8 constant.
  switch (info.type) {
    case AlertEventPublisher::AlertType::kFall:
      msg.alert_type = AiRuntimeAlert::ALERT_FALL;
      break;
    case AlertEventPublisher::AlertType::kFire:
      msg.alert_type = AiRuntimeAlert::ALERT_FIRE;
      break;
    case AlertEventPublisher::AlertType::kSmoke:
      msg.alert_type = AiRuntimeAlert::ALERT_SMOKE;
      break;
    case AlertEventPublisher::AlertType::kAnomaly:
      msg.alert_type = AiRuntimeAlert::ALERT_ANOMALY;
      break;
  }

  if (info.window_confidence >= 0.9f) {
    msg.level = AiRuntimeAlert::LEVEL_CRITICAL;
  } else if (info.window_confidence >= 0.7f) {
    msg.level = AiRuntimeAlert::LEVEL_WARNING;
  } else {
    msg.level = AiRuntimeAlert::LEVEL_INFO;
  }

  msg.alert_id = info.alert_id;
  msg.detector_source = info.detector_source;
  msg.confirmed_frames = static_cast<uint32_t>(info.confirmed_frames);
  msg.window_confidence = info.window_confidence;

  // Timestamps: use node's current ROS time (steady_clock → ROS time conversion
  // would need system_clock; for alert messages, now() is sufficient).
  msg.first_seen = node_->now();
  msg.last_seen = node_->now();

  pub_->publish(msg);
}

}  // namespace k1muse_ai_runtime
