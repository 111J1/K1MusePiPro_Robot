#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "k1muse_ai_runtime/alert_publisher.hpp"
#include "k1muse_ai_runtime_msgs/msg/airuntime_alert.hpp"

namespace k1muse_ai_runtime
{

/// ROS2 adapter that subscribes to AlertEventPublisher callbacks and
/// publishes AiRuntimeAlert messages on the /ai_runtime/alert topic.
class AlertRosPublisher
{
public:
  using AiRuntimeAlert = k1muse_ai_runtime_msgs::msg::AiruntimeAlert;

  /// Constructor accepts LifecycleNode (AiRuntimeNode type) or plain Node.
  explicit AlertRosPublisher(rclcpp::Node* node);

  /// Convert AlertInfo to AiRuntimeAlert and publish.
  void publish(const AlertEventPublisher::AlertInfo& info);

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<AiRuntimeAlert>::SharedPtr pub_;
};

}  // namespace k1muse_ai_runtime
