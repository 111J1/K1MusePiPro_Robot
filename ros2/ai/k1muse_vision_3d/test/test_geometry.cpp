#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_vision_3d/geometry.hpp"

using k1muse_vision_3d::geometry::BackProject;
using k1muse_vision_3d::geometry::CameraIntrinsics;
using k1muse_vision_3d::geometry::DepthImageView;
using k1muse_vision_3d::geometry::MedianDepthMeters;

// Test 1: MedianDepthFlat — flat depth image → correct median.
TEST(GeometryTest, MedianDepthFlat)
{
  // 3x3 image, all pixels at 2.0m (2000mm in 16UC1).
  DepthImageView depth;
  depth.width = 3;
  depth.height = 3;
  depth.encoding = "16UC1";
  depth.data = {
    0xD0, 0x07, 0xD0, 0x07, 0xD0, 0x07,  // 2000, 2000, 2000
    0xD0, 0x07, 0xD0, 0x07, 0xD0, 0x07,  // 2000, 2000, 2000
    0xD0, 0x07, 0xD0, 0x07, 0xD0, 0x07,  // 2000, 2000, 2000
  };

  const auto result = MedianDepthMeters(depth, 1.0, 1.0, 3, 0.1, 10.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.depth_m, 2.0F);
  EXPECT_EQ(result.reason, "ok");
}

// Test 2: MedianDepthOutOfRange — depth outside min/max → invalid.
TEST(GeometryTest, MedianDepthOutOfRange)
{
  // 3x3 image, all pixels at 5.0m (32FC1), but max_depth = 1.0m.
  DepthImageView depth;
  depth.width = 3;
  depth.height = 3;
  depth.encoding = "32FC1";

  const float val = 5.0f;
  depth.data.resize(9 * sizeof(float));
  for (int i = 0; i < 9; ++i) {
    std::memcpy(depth.data.data() + i * sizeof(float), &val, sizeof(float));
  }

  const auto result = MedianDepthMeters(depth, 1.0, 1.0, 3, 0.1, 1.0);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "no_valid_pixels");
}

// Test 3: MedianDepthNoValidPixels — all zeros → invalid.
TEST(GeometryTest, MedianDepthNoValidPixels)
{
  DepthImageView depth;
  depth.width = 3;
  depth.height = 3;
  depth.encoding = "16UC1";
  // All zeros = invalid pixels in 16UC1.
  depth.data = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  };

  const auto result = MedianDepthMeters(depth, 1.0, 1.0, 3, 0.1, 10.0);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "no_valid_pixels");
}

// Test 4: BackProjectCenter — center pixel → x≈0, y≈0.
TEST(GeometryTest, BackProjectCenter)
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;

  const auto point = BackProject(320.0, 240.0, 2.0, intrinsics);

  ASSERT_TRUE(point.valid);
  EXPECT_NEAR(point.x, 0.0F, 1e-5F);
  EXPECT_NEAR(point.y, 0.0F, 1e-5F);
  EXPECT_FLOAT_EQ(point.z, 2.0F);
  EXPECT_EQ(point.reason, "ok");
}

// Test 5: BackProjectOffCenter — off-center pixel → correct x, y.
TEST(GeometryTest, BackProjectOffCenter)
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;

  const auto point = BackProject(420.0, 140.0, 2.0, intrinsics);

  ASSERT_TRUE(point.valid);
  EXPECT_FLOAT_EQ(point.x, 0.4F);   // (420-320)*2/500 = 0.4
  EXPECT_FLOAT_EQ(point.y, -0.4F);  // (140-240)*2/500 = -0.4
  EXPECT_FLOAT_EQ(point.z, 2.0F);
  EXPECT_EQ(point.reason, "ok");
}

// Additional: BackProject with invalid intrinsics.
TEST(GeometryTest, BackProjectInvalidIntrinsics)
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = 0.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;

  const auto point = BackProject(420.0, 140.0, 2.0, intrinsics);

  EXPECT_FALSE(point.valid);
  EXPECT_EQ(point.reason, "invalid_intrinsics");
}

// Additional: BackProject with invalid depth.
TEST(GeometryTest, BackProjectInvalidDepth)
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;

  const auto point = BackProject(420.0, 140.0, -1.0, intrinsics);

  EXPECT_FALSE(point.valid);
  EXPECT_EQ(point.reason, "invalid_depth");
}

// Additional: MedianDepth with mixed valid/invalid 16UC1 values.
TEST(GeometryTest, MedianDepthMixed16UC1)
{
  // 3x3 image with mix of valid values and zeros (invalid).
  // Values: 0(invalid), 100mm, 2000mm, 500mm, 800mm, 600mm, 10000mm, 700mm, 900mm
  DepthImageView depth;
  depth.width = 3;
  depth.height = 3;
  depth.encoding = "16UC1";
  depth.data = {
    0x00, 0x00, 0x64, 0x00, 0xD0, 0x07,
    0xF4, 0x01, 0x20, 0x03, 0x58, 0x02,
    0x10, 0x27, 0x84, 0x03, 0xBC, 0x02,
  };

  const auto result = MedianDepthMeters(depth, 1.0, 1.0, 3, 0.2, 3.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.depth_m, 0.8F);
  EXPECT_EQ(result.reason, "ok");
}
