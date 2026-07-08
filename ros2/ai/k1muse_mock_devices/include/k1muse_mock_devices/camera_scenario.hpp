#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

namespace k1muse_mock_devices
{

struct CameraScenarioConfig
{
  uint32_t width = 640;
  uint32_t height = 480;
  double fx = 500.0;
  double fy = 500.0;
  double cx = 320.0;
  double cy = 240.0;
  std::string frame_id = "camera_color";
};

class CameraScenario
{
public:
  struct ImageFrame
  {
    std::vector<uint8_t> data;  // RGB8
    uint32_t width;
    uint32_t height;
    std::string encoding;
  };

  explicit CameraScenario(const CameraScenarioConfig & config);

  /// Generate a deterministic RGB gradient image.
  /// \param frame_id_counter  Frame counter used to offset the gradient.
  /// \return ImageFrame with RGB8 data.
  ImageFrame generate(uint32_t frame_id_counter) const;

  /// Generate CameraInfo matching the config.
  sensor_msgs::msg::CameraInfo generate_camera_info() const;

  const CameraScenarioConfig & config() const;

private:
  CameraScenarioConfig config_;
};

}  // namespace k1muse_mock_devices
