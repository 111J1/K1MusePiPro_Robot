#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

namespace k1muse_mock_devices
{

struct DepthScenarioConfig
{
  uint32_t width = 640;
  uint32_t height = 480;
  double fx = 500.0;
  double fy = 500.0;
  double cx = 320.0;
  double cy = 240.0;
  std::string encoding = "16UC1";  // mm
  double base_depth_m = 2.0;       // flat plane depth in meters
  std::string frame_id = "camera_depth";
};

class DepthScenario
{
public:
  struct DepthFrame
  {
    std::vector<uint8_t> data;
    uint32_t width;
    uint32_t height;
    std::string encoding;
  };

  explicit DepthScenario(const DepthScenarioConfig & config);

  /// Generate a flat depth plane at base_depth_m.
  /// \param frame_id_counter  Frame counter (unused for flat plane, but
  ///        kept for API consistency with CameraScenario).
  /// \return DepthFrame with 16UC1 data in millimeters.
  DepthFrame generate(uint32_t frame_id_counter) const;

  /// Generate CameraInfo matching the config.
  sensor_msgs::msg::CameraInfo generate_camera_info() const;

  const DepthScenarioConfig & config() const;

private:
  DepthScenarioConfig config_;
};

}  // namespace k1muse_mock_devices
