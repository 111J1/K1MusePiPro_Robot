#include <gtest/gtest.h>

#include "k1muse_common/qos_profiles.hpp"

TEST(QosProfiles, MatchFoundationContract)
{
  const auto audio = k1muse_common::qos::AudioStream();
  EXPECT_EQ(audio.depth(), 20U);
  EXPECT_EQ(audio.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(audio.durability(), rclcpp::DurabilityPolicy::Volatile);

  const auto sensor = k1muse_common::qos::SensorLatest();
  EXPECT_EQ(sensor.depth(), 3U);
  EXPECT_EQ(sensor.reliability(), rclcpp::ReliabilityPolicy::BestEffort);

  const auto event = k1muse_common::qos::ReliableEvent();
  EXPECT_EQ(event.depth(), 5U);
  EXPECT_EQ(event.reliability(), rclcpp::ReliabilityPolicy::Reliable);

  const auto result = k1muse_common::qos::ReliableResult();
  EXPECT_EQ(result.depth(), 10U);
  EXPECT_EQ(result.reliability(), rclcpp::ReliabilityPolicy::Reliable);

  const auto state = k1muse_common::qos::LatchedState();
  EXPECT_EQ(state.depth(), 1U);
  EXPECT_EQ(state.durability(), rclcpp::DurabilityPolicy::TransientLocal);

  const auto debug = k1muse_common::qos::DebugBestEffort();
  EXPECT_EQ(debug.depth(), 20U);
  EXPECT_EQ(debug.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
}
