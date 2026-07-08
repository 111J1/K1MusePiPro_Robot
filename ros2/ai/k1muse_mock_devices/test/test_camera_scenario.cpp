#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_mock_devices/camera_scenario.hpp"

using k1muse_mock_devices::CameraScenario;
using k1muse_mock_devices::CameraScenarioConfig;

namespace
{

constexpr uint32_t kWidth = 640;
constexpr uint32_t kHeight = 480;
constexpr double kFx = 500.0;
constexpr double kFy = 500.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;
const std::string kFrameId = "camera_color";

CameraScenarioConfig default_config()
{
  CameraScenarioConfig cfg;
  cfg.width = kWidth;
  cfg.height = kHeight;
  cfg.fx = kFx;
  cfg.fy = kFy;
  cfg.cx = kCx;
  cfg.cy = kCy;
  cfg.frame_id = kFrameId;
  return cfg;
}

}  // namespace

// ---------------------------------------------------------------------------
// GenerateImage: correct size (width * height * 3)
// ---------------------------------------------------------------------------

TEST(CameraScenario, GenerateImage)
{
  CameraScenario scenario(default_config());
  auto frame = scenario.generate(0);

  EXPECT_EQ(frame.width, kWidth);
  EXPECT_EQ(frame.height, kHeight);
  EXPECT_EQ(frame.encoding, "rgb8");
  EXPECT_EQ(frame.data.size(), kWidth * kHeight * 3);
}

// ---------------------------------------------------------------------------
// GenerateCameraInfo: intrinsics match config
// ---------------------------------------------------------------------------

TEST(CameraScenario, GenerateCameraInfo)
{
  CameraScenarioConfig cfg = default_config();
  CameraScenario scenario(cfg);
  auto info = scenario.generate_camera_info();

  EXPECT_EQ(info.width, kWidth);
  EXPECT_EQ(info.height, kHeight);
  EXPECT_EQ(info.header.frame_id, kFrameId);
  EXPECT_EQ(info.distortion_model, "plumb_bob");

  // K matrix: fx, fy, cx, cy
  EXPECT_DOUBLE_EQ(info.k[0], kFx);   // K[0][0]
  EXPECT_DOUBLE_EQ(info.k[2], kCx);   // K[0][2]
  EXPECT_DOUBLE_EQ(info.k[4], kFy);   // K[1][1]
  EXPECT_DOUBLE_EQ(info.k[5], kCy);   // K[1][2]
  EXPECT_DOUBLE_EQ(info.k[8], 1.0);   // K[2][2]

  // P matrix: fx, fy, cx, cy
  EXPECT_DOUBLE_EQ(info.p[0], kFx);   // P[0][0]
  EXPECT_DOUBLE_EQ(info.p[2], kCx);   // P[0][2]
  EXPECT_DOUBLE_EQ(info.p[5], kFy);   // P[1][1]
  EXPECT_DOUBLE_EQ(info.p[6], kCy);   // P[1][2]
  EXPECT_DOUBLE_EQ(info.p[10], 1.0);  // P[2][2]
}

// ---------------------------------------------------------------------------
// DeterministicSameFrame: same frame_id -> same image
// ---------------------------------------------------------------------------

TEST(CameraScenario, DeterministicSameFrame)
{
  CameraScenario scenario(default_config());

  auto frame1 = scenario.generate(42);
  auto frame2 = scenario.generate(42);

  EXPECT_EQ(frame1.data.size(), frame2.data.size());
  EXPECT_EQ(frame1.data, frame2.data);
}

// ---------------------------------------------------------------------------
// DifferentFrames: different frame_id -> different image (B channel changes)
// ---------------------------------------------------------------------------

TEST(CameraScenario, DifferentFrames)
{
  CameraScenario scenario(default_config());

  auto frame1 = scenario.generate(0);
  auto frame2 = scenario.generate(1);

  ASSERT_EQ(frame1.data.size(), frame2.data.size());

  // At least some pixels should differ (B channel depends on frame counter)
  bool has_diff = false;
  for (std::size_t i = 0; i < frame1.data.size(); ++i) {
    if (frame1.data[i] != frame2.data[i]) {
      has_diff = true;
      break;
    }
  }
  EXPECT_TRUE(has_diff);
}

// ---------------------------------------------------------------------------
// GradientPattern: verify R and G channels follow expected gradient
// ---------------------------------------------------------------------------

TEST(CameraScenario, GradientPattern)
{
  CameraScenarioConfig cfg;
  cfg.width = 4;
  cfg.height = 4;
  cfg.fx = 500.0;
  cfg.fy = 500.0;
  cfg.cx = 2.0;
  cfg.cy = 2.0;
  cfg.frame_id = "test";

  CameraScenario scenario(cfg);
  auto frame = scenario.generate(0);

  // Check first pixel (0,0): R=0, G=0
  EXPECT_EQ(frame.data[0], 0);  // R at (0,0)
  EXPECT_EQ(frame.data[1], 0);  // G at (0,0)

  // Check last pixel (3,3): R=255, G=255
  const std::size_t last_idx = (3 * 4 + 3) * 3;
  EXPECT_EQ(frame.data[last_idx + 0], 255);  // R at (3,3)
  EXPECT_EQ(frame.data[last_idx + 1], 255);  // G at (3,3)

  // Check pixel (3,0): R=255, G=0
  const std::size_t right_top = (0 * 4 + 3) * 3;
  EXPECT_EQ(frame.data[right_top + 0], 255);  // R at (3,0)
  EXPECT_EQ(frame.data[right_top + 1], 0);    // G at (3,0)

  // Check pixel (0,3): R=0, G=255
  const std::size_t left_bottom = (3 * 4 + 0) * 3;
  EXPECT_EQ(frame.data[left_bottom + 0], 0);    // R at (0,3)
  EXPECT_EQ(frame.data[left_bottom + 1], 255);  // G at (0,3)
}
