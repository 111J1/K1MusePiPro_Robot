#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace k1muse_vision_3d::geometry
{

struct DepthImageView
{
  int width = 0;
  int height = 0;
  std::string encoding;
  std::vector<std::uint8_t> data;
};

struct DepthResult
{
  bool valid = false;
  float depth_m = 0.0F;
  std::string reason;  // "ok", "no_valid_pixels", "out_of_range"
};

struct CameraIntrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

struct Point3DResult
{
  bool valid = false;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::string reason;  // "ok", "invalid_intrinsics", "invalid_depth"
};

DepthResult MedianDepthMeters(
  const DepthImageView & image,
  double center_u,
  double center_v,
  int window_px,
  double min_depth_m,
  double max_depth_m);

Point3DResult BackProject(
  double u,
  double v,
  double depth_m,
  const CameraIntrinsics & intrinsics);

}  // namespace k1muse_vision_3d::geometry
