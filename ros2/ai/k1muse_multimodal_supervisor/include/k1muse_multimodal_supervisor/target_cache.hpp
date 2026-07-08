#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

namespace k1muse_multimodal_supervisor {

/// A single detection cached from a Detection2D frame.
struct CachedDetection {
  std::string detection_id;
  std::string class_name;
  float score = 0.0f;
  uint32_t x = 0;
  uint32_t y = 0;
  uint32_t width = 0;
  uint32_t height = 0;
};

/// Result of a target lookup against the cache.
struct TargetResult {
  bool found = false;
  std::string target_id;
  std::string target_class;
  float score = 0.0f;
  std::string reason;
};

/// Time-bounded cache of the latest detection frame.
/// Thread-safe: all public methods acquire an internal mutex.
class TargetCache {
public:
  explicit TargetCache(
      std::chrono::milliseconds ttl = std::chrono::milliseconds(500));

  /// Update the cache with a new detection frame.
  void update(uint32_t image_width, uint32_t image_height,
              std::vector<CachedDetection> detections,
              std::chrono::steady_clock::time_point stamp);

  /// Find the best matching target.  Returns TargetResult.
  TargetResult find_target(const std::string& target_class,
                           float minimum_score = 0.0f) const;

  /// Check if the cache has a valid (non-expired) frame.
  bool has_valid_frame() const;

  /// Clear the cache.
  void clear();

private:
  bool is_expired() const;

  std::chrono::milliseconds ttl_;
  mutable std::mutex mutex_;
  std::vector<CachedDetection> detections_;
  uint32_t image_width_ = 0;
  uint32_t image_height_ = 0;
  std::chrono::steady_clock::time_point stamp_;
  bool has_frame_ = false;
};

}  // namespace k1muse_multimodal_supervisor
