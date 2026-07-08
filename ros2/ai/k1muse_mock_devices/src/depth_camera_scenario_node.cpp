#include "k1muse_mock_devices/depth_scenario.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "k1muse_common/qos_profiles.hpp"

namespace k1muse_mock_devices
{

class DepthCameraScenarioNode : public rclcpp::Node
{
public:
  explicit DepthCameraScenarioNode(const rclcpp::NodeOptions & options)
  : Node("depth_camera_scenario_node", options)
  {
    // Declare parameters
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<double>("fx", 500.0);
    this->declare_parameter<double>("fy", 500.0);
    this->declare_parameter<double>("cx", 320.0);
    this->declare_parameter<double>("cy", 240.0);
    this->declare_parameter<std::string>("encoding", "16UC1");
    this->declare_parameter<double>("base_depth_m", 2.0);
    this->declare_parameter<std::string>("frame_id", "camera_depth");
    this->declare_parameter<int>("publish_rate_ms", 33);  // ~30 fps

    // Read parameters
    DepthScenarioConfig cfg;
    cfg.width = static_cast<uint32_t>(
      this->get_parameter("width").as_int());
    cfg.height = static_cast<uint32_t>(
      this->get_parameter("height").as_int());
    cfg.fx = this->get_parameter("fx").as_double();
    cfg.fy = this->get_parameter("fy").as_double();
    cfg.cx = this->get_parameter("cx").as_double();
    cfg.cy = this->get_parameter("cy").as_double();
    cfg.encoding = this->get_parameter("encoding").as_string();
    cfg.base_depth_m = this->get_parameter("base_depth_m").as_double();
    cfg.frame_id = this->get_parameter("frame_id").as_string();
    publish_rate_ms_ = static_cast<uint16_t>(
      this->get_parameter("publish_rate_ms").as_int());

    scenario_ = std::make_unique<DepthScenario>(cfg);
    frame_counter_ = 0;

    // Publishers
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/main/depth_registered/image_raw",
      k1muse_common::qos::SensorLatest(3));

    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/main/depth_registered/camera_info",
      k1muse_common::qos::SensorLatest(3));

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_rate_ms_),
      [this]() { publish_frame(); });

    RCLCPP_INFO(this->get_logger(),
      "DepthCameraScenarioNode started: %ux%u fx=%.1f fy=%.1f "
      "cx=%.1f cy=%.1f encoding=%s base_depth=%.2fm frame=%s rate=%ums",
      cfg.width, cfg.height, cfg.fx, cfg.fy,
      cfg.cx, cfg.cy, cfg.encoding.c_str(),
      cfg.base_depth_m, cfg.frame_id.c_str(),
      static_cast<unsigned>(publish_rate_ms_));
  }

private:
  std::unique_ptr<DepthScenario> scenario_;
  uint32_t frame_counter_{0};
  uint16_t publish_rate_ms_{33};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_frame()
  {
    const auto & cfg = scenario_->config();

    // Generate depth image
    auto depth_frame = scenario_->generate(frame_counter_);

    auto img_msg = sensor_msgs::msg::Image();
    img_msg.header.stamp = this->now();
    img_msg.header.frame_id = cfg.frame_id;
    img_msg.height = depth_frame.height;
    img_msg.width = depth_frame.width;
    img_msg.encoding = depth_frame.encoding;
    img_msg.is_bigendian = 0;

    // Step size depends on encoding
    if (cfg.encoding == "16UC1") {
      img_msg.step = depth_frame.width * 2;
    } else if (cfg.encoding == "32FC1") {
      img_msg.step = depth_frame.width * 4;
    } else {
      img_msg.step = depth_frame.width * 2;
    }
    img_msg.data = std::move(depth_frame.data);

    // Generate CameraInfo
    auto info_msg = scenario_->generate_camera_info();
    info_msg.header.stamp = img_msg.header.stamp;

    image_pub_->publish(img_msg);
    info_pub_->publish(info_msg);

    ++frame_counter_;
  }
};

}  // namespace k1muse_mock_devices

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_mock_devices::DepthCameraScenarioNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
