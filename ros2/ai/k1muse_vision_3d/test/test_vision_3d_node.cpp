#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "k1muse_vision_3d/vision_3d_node.hpp"
#include "k1muse_vision_msgs/msg/detection2_d.hpp"
#include "k1muse_vision_msgs/msg/detection2_d_frame.hpp"
#include "k1muse_vision_msgs/msg/target3_d.hpp"
#include "k1muse_vision_msgs/msg/target_response.hpp"

namespace k1muse_vision_3d
{
namespace
{

using Detection2D = k1muse_vision_msgs::msg::Detection2D;
using Detection2DFrame = k1muse_vision_msgs::msg::Detection2DFrame;
using Target3D = k1muse_vision_msgs::msg::Target3D;
using TargetResponse = k1muse_vision_msgs::msg::TargetResponse;

// ---- Helpers ----

template <typename T>
class MessageCollector
{
public:
  explicit MessageCollector(
    rclcpp::Node::SharedPtr node, const std::string & topic,
    const rclcpp::QoS & qos = rclcpp::QoS(10).reliable().durability_volatile())
  {
    sub_ = node->create_subscription<T>(
      topic, qos,
      [this](typename T::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        msgs_.push_back(*msg);
        cv_.notify_all();
      });
  }

  bool wait_for(size_t count, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_.wait_for(lock, timeout, [this, count]() {
      return msgs_.size() >= count;
    });
  }

  const std::vector<T> & messages() const { return msgs_; }
  size_t size() const { return msgs_.size(); }

private:
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  std::vector<T> msgs_;
  std::mutex mtx_;
  std::condition_variable cv_;
};

sensor_msgs::msg::Image make_depth_image_32fc1(
  int width, int height, float fill_value)
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "depth_camera_optical_frame";
  msg.width = width;
  msg.height = height;
  msg.encoding = "32FC1";
  msg.is_bigendian = false;
  msg.step = width * sizeof(float);
  msg.data.resize(width * height * sizeof(float));
  for (int i = 0; i < width * height; ++i) {
    std::memcpy(msg.data.data() + i * sizeof(float), &fill_value, sizeof(float));
  }
  return msg;
}

sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "depth_camera_optical_frame";
  msg.k[0] = 500.0;  // fx
  msg.k[2] = 320.0;  // cx
  msg.k[4] = 500.0;  // fy
  msg.k[5] = 240.0;  // cy
  return msg;
}

Detection2DFrame make_detection_frame(
  const std::string & det_id,
  uint32_t x, uint32_t y, uint32_t w, uint32_t h,
  rclcpp::Time stamp = rclcpp::Clock().now())
{
  Detection2DFrame frame;
  frame.header.stamp = stamp;
  frame.trace_id = "test_trace";
  frame.epoch = 1;

  Detection2D det;
  det.detection_id = det_id;
  det.class_name = "target";
  det.score = 0.9f;
  det.x = x;
  det.y = y;
  det.width = w;
  det.height = h;
  frame.detections.push_back(det);

  return frame;
}

TargetResponse make_target_response(
  const std::string & target_id, bool found = true)
{
  TargetResponse msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.request_id = "req1";
  msg.trace_id = "test_trace";
  msg.epoch = 1;
  msg.found = found;
  msg.target_id = target_id;
  msg.target_class = "target";
  msg.score = 0.9f;
  return msg;
}

// ---- Global test environment ----

class Vision3DNodeTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

::testing::Environment * const kEnv =
  ::testing::AddGlobalTestEnvironment(new Vision3DNodeTestEnvironment);

// ---- Test fixture ----

class Vision3DNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<Vision3DNode>(rclcpp::NodeOptions());
    test_node_ = std::make_shared<rclcpp::Node>("test_vision_3d_helper");
    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = test_node_->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "depth_camera_optical_frame";
    tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(test_node_->get_node_base_interface());
    spin_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_future_.valid()) {
      spin_future_.wait();
    }
    executor_->remove_node(node_);
    executor_->remove_node(test_node_);
    node_.reset();
    test_node_.reset();
    executor_.reset();
  }

  std::shared_ptr<Vision3DNode> node_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::future<void> spin_future_;
};

// ---- Tests ----

// Test 1: TargetFoundWithValidDepth — detection + depth → Target3D valid.
TEST_F(Vision3DNodeTest, TargetFoundWithValidDepth)
{
  // Create test publishers.
  auto det_pub = test_node_->create_publisher<Detection2DFrame>(
    "/vision/detection_2d", rclcpp::QoS(5).reliable().durability_volatile());
  auto depth_pub = test_node_->create_publisher<sensor_msgs::msg::Image>(
    "/camera/main/depth_registered/image_raw", rclcpp::QoS(3).best_effort().durability_volatile());
  auto info_pub = test_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/main/color/camera_info", rclcpp::QoS(3).best_effort().durability_volatile());
  auto resp_pub = test_node_->create_publisher<TargetResponse>(
    "/vision/target_response", rclcpp::QoS(5).reliable().durability_volatile());

  // Subscribe to output.
  MessageCollector<Target3D> target_col(test_node_, "/vision/main/target_3d");

  // Publish detection frame: detection at (40,40) size (40,40), center=(60,60).
  det_pub->publish(make_detection_frame("det_001", 40, 40, 40, 40));

  // Publish depth image: 100x100, all 1.5m.
  depth_pub->publish(make_depth_image_32fc1(100, 100, 1.5f));

  // Publish camera info: fx=fy=500, cx=320, cy=240.
  info_pub->publish(make_camera_info());

  // Allow subscribers to receive.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Publish target response.
  resp_pub->publish(make_target_response("det_001"));

  // Wait for output.
  ASSERT_TRUE(target_col.wait_for(1, std::chrono::milliseconds(3000)));

  const auto & target = target_col.messages()[0];
  EXPECT_TRUE(target.valid);
  EXPECT_EQ(target.reason, "ok");
  EXPECT_EQ(target.header.frame_id, "base_link");
  EXPECT_EQ(target.target_id, "det_001");
  EXPECT_FLOAT_EQ(target.depth, 1.5F);

  // center_u=60, center_v=60, depth=1.5, fx=500, fy=500, cx=320, cy=240
  // x = (60-320)*1.5/500 = -0.78
  // y = (60-240)*1.5/500 = -0.54
  EXPECT_NEAR(target.x, -0.78F, 0.01F);
  EXPECT_NEAR(target.y, -0.54F, 0.01F);
  EXPECT_FLOAT_EQ(target.z, 1.5F);
}

// Test 2: DetectionNotFound — wrong target_id → valid=false.
TEST_F(Vision3DNodeTest, DetectionNotFound)
{
  auto det_pub = test_node_->create_publisher<Detection2DFrame>(
    "/vision/detection_2d", rclcpp::QoS(5).reliable().durability_volatile());
  auto resp_pub = test_node_->create_publisher<TargetResponse>(
    "/vision/target_response", rclcpp::QoS(5).reliable().durability_volatile());

  MessageCollector<Target3D> target_col(test_node_, "/vision/main/target_3d");

  // Publish detection with id "det_001".
  det_pub->publish(make_detection_frame("det_001", 40, 40, 40, 40));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Request a different id.
  resp_pub->publish(make_target_response("det_999"));

  ASSERT_TRUE(target_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_FALSE(target_col.messages()[0].valid);
  EXPECT_EQ(target_col.messages()[0].reason, "detection_not_found");
}

// Test 3: ExpiredFrame — old detection → valid=false, reason="frame_expired".
TEST_F(Vision3DNodeTest, ExpiredFrame)
{
  auto det_pub = test_node_->create_publisher<Detection2DFrame>(
    "/vision/detection_2d", rclcpp::QoS(5).reliable().durability_volatile());
  auto depth_pub = test_node_->create_publisher<sensor_msgs::msg::Image>(
    "/camera/main/depth_registered/image_raw", rclcpp::QoS(3).best_effort().durability_volatile());
  auto info_pub = test_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/main/color/camera_info", rclcpp::QoS(3).best_effort().durability_volatile());
  auto resp_pub = test_node_->create_publisher<TargetResponse>(
    "/vision/target_response", rclcpp::QoS(5).reliable().durability_volatile());

  MessageCollector<Target3D> target_col(test_node_, "/vision/main/target_3d");

  // Publish detection frame with an old timestamp (10 seconds ago).
  auto old_stamp = rclcpp::Clock().now() - rclcpp::Duration::from_seconds(10.0);
  det_pub->publish(make_detection_frame("det_001", 40, 40, 40, 40, old_stamp));

  // Publish depth and camera info.
  depth_pub->publish(make_depth_image_32fc1(100, 100, 1.5f));
  info_pub->publish(make_camera_info());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Publish target response.
  resp_pub->publish(make_target_response("det_001"));

  ASSERT_TRUE(target_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_FALSE(target_col.messages()[0].valid);
  EXPECT_EQ(target_col.messages()[0].reason, "frame_expired");
}

}  // namespace
}  // namespace k1muse_vision_3d
