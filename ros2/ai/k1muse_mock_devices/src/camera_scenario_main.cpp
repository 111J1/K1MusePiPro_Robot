#include "k1muse_mock_devices/camera_scenario.hpp"

#include <cstdint>
#include <vector>

namespace k1muse_mock_devices
{

CameraScenario::CameraScenario(const CameraScenarioConfig & config)
: config_(config) {}

CameraScenario::ImageFrame CameraScenario::generate(
  uint32_t frame_id_counter) const
{
  ImageFrame frame;
  frame.width = config_.width;
  frame.height = config_.height;
  frame.encoding = "rgb8";

  const std::size_t num_pixels = config_.width * config_.height;
  frame.data.resize(num_pixels * 3);

  // Deterministic RGB gradient: R increases left-to-right, G increases
  // top-to-bottom, B is a function of both plus the frame counter.
  for (uint32_t y = 0; y < config_.height; ++y) {
    for (uint32_t x = 0; x < config_.width; ++x) {
      const std::size_t idx = (y * config_.width + x) * 3;
      uint8_t r = static_cast<uint8_t>(
        (x * 255) / (config_.width - 1 == 0 ? 1 : config_.width - 1));
      uint8_t g = static_cast<uint8_t>(
        (y * 255) / (config_.height - 1 == 0 ? 1 : config_.height - 1));
      uint8_t b = static_cast<uint8_t>((frame_id_counter + x + y) % 256);
      frame.data[idx + 0] = r;
      frame.data[idx + 1] = g;
      frame.data[idx + 2] = b;
    }
  }

  return frame;
}

sensor_msgs::msg::CameraInfo CameraScenario::generate_camera_info() const
{
  sensor_msgs::msg::CameraInfo info;
  info.width = config_.width;
  info.height = config_.height;
  info.header.frame_id = config_.frame_id;

  // Distortion model
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  // Intrinsic matrix K (3x3, row-major)
  //   fx  0  cx
  //    0 fy  cy
  //    0  0   1
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
  //   fx  0  cx  0
  //    0 fy  cy  0
  //    0  0   1  0
  info.p = {{
    config_.fx, 0.0, config_.cx, 0.0,
    0.0, config_.fy, config_.cy, 0.0,
    0.0, 0.0, 1.0, 0.0
  }};

  return info;
}

const CameraScenarioConfig & CameraScenario::config() const
{
  return config_;
}

}  // namespace k1muse_mock_devices
