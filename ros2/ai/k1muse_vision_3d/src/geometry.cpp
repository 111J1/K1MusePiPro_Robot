#include "k1muse_vision_3d/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace k1muse_vision_3d::geometry
{
namespace
{

bool ReadDepthAt(const DepthImageView & image, const int x, const int y, double * depth_m)
{
  if (x < 0 || y < 0 || x >= image.width || y >= image.height || depth_m == nullptr) {
    return false;
  }

  if (image.encoding == "16UC1") {
    const std::size_t offset =
      static_cast<std::size_t>(y * image.width + x) * sizeof(std::uint16_t);
    if (offset + sizeof(std::uint16_t) > image.data.size()) {
      return false;
    }
    std::uint16_t raw = 0;
    std::memcpy(&raw, image.data.data() + offset, sizeof(raw));
    if (raw == 0) {
      return false;
    }
    *depth_m = static_cast<double>(raw) / 1000.0;
    return true;
  }

  if (image.encoding == "32FC1") {
    const std::size_t offset =
      static_cast<std::size_t>(y * image.width + x) * sizeof(float);
    if (offset + sizeof(float) > image.data.size()) {
      return false;
    }
    float raw = 0.0F;
    std::memcpy(&raw, image.data.data() + offset, sizeof(raw));
    *depth_m = static_cast<double>(raw);
    return true;
  }

  return false;
}

}  // namespace

DepthResult MedianDepthMeters(
  const DepthImageView & image,
  const double center_u,
  const double center_v,
  const int window_px,
  const double min_depth_m,
  const double max_depth_m)
{
  DepthResult result;
  if (image.width <= 0 || image.height <= 0 || window_px <= 0) {
    return result;
  }

  const int center_x = static_cast<int>(std::llround(center_u));
  const int center_y = static_cast<int>(std::llround(center_v));
  const int half = window_px / 2;
  const int min_x = std::max(0, center_x - half);
  const int max_x = std::min(image.width - 1, center_x + half);
  const int min_y = std::max(0, center_y - half);
  const int max_y = std::min(image.height - 1, center_y + half);

  std::vector<double> values;
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      double depth = 0.0;
      if (!ReadDepthAt(image, x, y, &depth)) {
        continue;
      }
      if (std::isfinite(depth) && depth >= min_depth_m && depth <= max_depth_m) {
        values.push_back(depth);
      }
    }
  }

  if (values.empty()) {
    result.reason = "no_valid_pixels";
    return result;
  }
  std::sort(values.begin(), values.end());
  const float median = static_cast<float>(values[values.size() / 2]);
  if (median < min_depth_m || median > max_depth_m) {
    result.depth_m = median;
    result.reason = "out_of_range";
    return result;
  }
  result.valid = true;
  result.depth_m = median;
  result.reason = "ok";
  return result;
}

Point3DResult BackProject(
  const double u,
  const double v,
  const double depth_m,
  const CameraIntrinsics & intrinsics)
{
  Point3DResult result;
  if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
    result.reason = "invalid_intrinsics";
    return result;
  }
  if (depth_m <= 0.0 || !std::isfinite(depth_m)) {
    result.reason = "invalid_depth";
    return result;
  }
  result.valid = true;
  result.x = static_cast<float>((u - intrinsics.cx) * depth_m / intrinsics.fx);
  result.y = static_cast<float>((v - intrinsics.cy) * depth_m / intrinsics.fy);
  result.z = static_cast<float>(depth_m);
  result.reason = "ok";
  return result;
}

}  // namespace k1muse_vision_3d::geometry
