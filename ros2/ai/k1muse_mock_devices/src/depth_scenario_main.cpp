#include "k1muse_mock_devices/depth_scenario.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

namespace k1muse_mock_devices
{

DepthScenario::DepthScenario(const DepthScenarioConfig & config)
: config_(config) {}

DepthScenario::DepthFrame DepthScenario::generate(
  uint32_t /*frame_id_counter*/) const
{
  DepthFrame frame;
  frame.width = config_.width;
  frame.height = config_.height;
  frame.encoding = config_.encoding;

  const std::size_t num_pixels = config_.width * config_.height;

  if (config_.encoding == "16UC1") {
    // 16-bit unsigned, millimeters
    const uint16_t depth_mm = static_cast<uint16_t>(config_.base_depth_m * 1000.0);
    frame.data.resize(num_pixels * sizeof(uint16_t));
    for (std::size_t i = 0; i < num_pixels; ++i) {
      std::memcpy(&frame.data[i * sizeof(uint16_t)], &depth_mm,
                   sizeof(uint16_t));
    }
  } else if (config_.encoding == "32FC1") {
    // 32-bit float, meters
    const float depth_m = static_cast<float>(config_.base_depth_m);
    frame.data.resize(num_pixels * sizeof(float));
    for (std::size_t i = 0; i < num_pixels; ++i) {
      std::memcpy(&frame.data[i * sizeof(float)], &depth_m, sizeof(float));
    }
  } else {
    // Default: fill with zeros
    frame.data.resize(num_pixels * 2, 0);
  }

  return frame;
}

sensor_msgs::msg::CameraInfo DepthScenario::generate_camera_info() const
{
  sensor_msgs::msg::CameraInfo info;
  info.width = config_.width;
  info.height = config_.height;
  info.header.frame_id = config_.frame_id;

  // Distortion model
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  // Intrinsic matrix K (3x3, row-major)
  info.k = {{
    config_.fx, 0.0, config_.cx,
    0.0, config_.fy, config_.cy,
    0.0, 0.0, 1.0
  }};

  // Rectification matrix R (identity)
  info.r = {{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  }};

  // Projection matrix P (3x4)
  info.p = {{
    config_.fx, 0.0, config_.cx, 0.0,
    0.0, config_.fy, config_.cy, 0.0,
    0.0, 0.0, 1.0, 0.0
  }};

  return info;
}

const DepthScenarioConfig & DepthScenario::config() const
{
  return config_;
}

}  // namespace k1muse_mock_devices
