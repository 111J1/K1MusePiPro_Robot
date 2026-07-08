#include "k1muse_multimodal_supervisor/target_cache.hpp"

#include <algorithm>
#include <limits>

namespace k1muse_multimodal_supervisor {

TargetCache::TargetCache(std::chrono::milliseconds ttl) : ttl_(ttl) {}

// ---------------------------------------------------------------------------
// update
// ---------------------------------------------------------------------------
void TargetCache::update(uint32_t image_width, uint32_t image_height,
                         std::vector<CachedDetection> detections,
                         std::chrono::steady_clock::time_point stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  image_width_  = image_width;
  image_height_ = image_height;
  detections_   = std::move(detections);
  stamp_        = stamp;
  has_frame_    = true;
}

// ---------------------------------------------------------------------------
// find_target
// ---------------------------------------------------------------------------
TargetResult TargetCache::find_target(const std::string& target_class,
                                      float minimum_score) const {
  std::lock_guard<std::mutex> lock(mutex_);

  TargetResult result;

  if (!has_frame_) {
    result.reason = "no frame";
    return result;
  }

  if (is_expired()) {
    result.found  = false;
    result.reason = "expired";
    return result;
  }

  // Find the detection with the highest score that matches the class and
  // meets the minimum score threshold.
  float best_score = -1.0f;
  std::size_t best_idx = std::numeric_limits<std::size_t>::max();

  for (std::size_t i = 0; i < detections_.size(); ++i) {
    const auto& d = detections_[i];
    if (d.class_name == target_class && d.score >= minimum_score &&
        d.score > best_score) {
      best_score = d.score;
      best_idx   = i;
    }
  }

  if (best_idx < detections_.size()) {
    const auto& d = detections_[best_idx];
    result.found       = true;
    result.target_id   = d.detection_id;
    result.target_class = d.class_name;
    result.score       = d.score;
    result.reason      = "matched";
  } else {
    result.found  = false;
    result.reason = "no matching class";
  }

  return result;
}

// ---------------------------------------------------------------------------
// has_valid_frame
// ---------------------------------------------------------------------------
bool TargetCache::has_valid_frame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_frame_ && !is_expired();
}

// ---------------------------------------------------------------------------
// clear
// ---------------------------------------------------------------------------
void TargetCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  detections_.clear();
  image_width_  = 0;
  image_height_ = 0;
  has_frame_    = false;
}

// ---------------------------------------------------------------------------
// is_expired  (private, called under lock)
// ---------------------------------------------------------------------------
bool TargetCache::is_expired() const {
  auto now = std::chrono::steady_clock::now();
  return (now - stamp_) > ttl_;
}

}  // namespace k1muse_multimodal_supervisor
