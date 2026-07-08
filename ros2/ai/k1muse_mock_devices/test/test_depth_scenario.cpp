#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_mock_devices/depth_scenario.hpp"

using k1muse_mock_devices::DepthScenario;
using k1muse_mock_devices::DepthScenarioConfig;

namespace
{

constexpr uint32_t kWidth = 640;
constexpr uint32_t kHeight = 480;
constexpr double kFx = 500.0;
constexpr double kFy = 500.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;
constexpr double kBaseDepthM = 2.0;
const std::string kEncoding = "16UC1";
const std::string kFrameId = "camera_depth";

DepthScenarioConfig default_config()
{
  DepthScenarioConfig cfg;
  cfg.width = kWidth;
  cfg.height = kHeight;
  cfg.fx = kFx;
  cfg.fy = kFy;
  cfg.cx = kCx;
  cfg.cy = kCy;
  cfg.encoding = kEncoding;
  cfg.base_depth_m = kBaseDepthM;
  cfg.frame_id = kFrameId;
  return cfg;
}

}  // namespace

// ---------------------------------------------------------------------------
// GenerateDepth: correct size, encoding "16UC1"
// ---------------------------------------------------------------------------

TEST(DepthScenario, GenerateDepth)
{
  DepthScenario scenario(default_config());
  auto frame = scenario.generate(0);

  EXPECT_EQ(frame.width, kWidth);
  EXPECT_EQ(frame.height, kHeight);
  EXPECT_EQ(frame.encoding, "16UC1");

  // 16UC1: 2 bytes per pixel
  EXPECT_EQ(frame.data.size(), kWidth * kHeight * 2);
}

// ---------------------------------------------------------------------------
// FlatPlaneDepth: all pixels at base_depth_m (in mm for 16UC1)
// ---------------------------------------------------------------------------

TEST(DepthScenario, FlatPlaneDepth)
{
  DepthScenario scenario(default_config());
  auto frame = scenario.generate(0);

  // base_depth_m = 2.0 -> 2000 mm
  const uint16_t expected_mm = 2000;

  const std::size_t num_pixels = kWidth * kHeight;
  for (std::size_t i = 0; i < num_pixels; ++i) {
    uint16_t pixel_value;
    std::memcpy(&pixel_value, &frame.data[i * sizeof(uint16_t)],
                 sizeof(uint16_t));
    EXPECT_EQ(pixel_value, expected_mm)
      << "Pixel " << i << " depth mismatch";
  }
}

// ---------------------------------------------------------------------------
// GenerateCameraInfo: intrinsics match config
// ---------------------------------------------------------------------------

TEST(DepthScenario, GenerateCameraInfo)
{
  DepthScenarioConfig cfg = default_config();
  DepthScenario scenario(cfg);
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
// DifferentDepthValues: verify various base depths
// ---------------------------------------------------------------------------

TEST(DepthScenario, DifferentDepthValues)
{
  DepthScenarioConfig cfg = default_config();
  cfg.base_depth_m = 5.0;
  DepthScenario scenario(cfg);
  auto frame = scenario.generate(0);

  const uint16_t expected_mm = 5000;
  const std::size_t num_pixels = kWidth * kHeight;
  for (std::size_t i = 0; i < num_pixels; ++i) {
    uint16_t pixel_value;
    std::memcpy(&pixel_value, &frame.data[i * sizeof(uint16_t)],
                 sizeof(uint16_t));
    EXPECT_EQ(pixel_value, expected_mm)
      << "Pixel " << i << " depth mismatch for 5.0m";
  }
}

// ---------------------------------------------------------------------------
// Deterministic: same frame counter produces same data
// ---------------------------------------------------------------------------

TEST(DepthScenario, DeterministicSameFrame)
{
  DepthScenario scenario(default_config());

  auto frame1 = scenario.generate(0);
  auto frame2 = scenario.generate(42);

  // Flat plane should be identical regardless of frame counter
  EXPECT_EQ(frame1.data, frame2.data);
}
