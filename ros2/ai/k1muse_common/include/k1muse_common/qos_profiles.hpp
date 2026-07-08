#pragma once

#include <cstddef>

#include "rclcpp/qos.hpp"

namespace k1muse_common::qos
{

inline rclcpp::QoS AudioStream(std::size_t depth = 20)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

inline rclcpp::QoS SensorLatest(std::size_t depth = 3)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
}

inline rclcpp::QoS ReliableEvent(std::size_t depth = 5)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

inline rclcpp::QoS ReliableResult(std::size_t depth = 10)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

inline rclcpp::QoS LatchedState(std::size_t depth = 1)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

inline rclcpp::QoS DebugBestEffort(std::size_t depth = 20)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
}

}  // namespace k1muse_common::qos
