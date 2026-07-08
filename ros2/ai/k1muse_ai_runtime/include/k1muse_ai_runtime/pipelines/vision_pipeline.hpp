#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "k1muse_ai_runtime/alert_publisher.hpp"
#include "k1muse_ai_runtime/backends/vision_backend.hpp"
#include "k1muse_ai_runtime/control_snapshot.hpp"
#include "k1muse_ai_runtime/detector_registry.hpp"
#include "k1muse_ai_runtime/inference_job.hpp"
#include "k1muse_ai_runtime/runtime_profile_manager.hpp"
#include "k1muse_ai_runtime/vision/latest_frame_buffer.hpp"
#include "k1muse_ai_runtime/vision_detector_scheduler.hpp"
#include "k1muse_ai_runtime/voice_exclusive_guard.hpp"

namespace k1muse_ai_runtime
{

/// Vision pipeline orchestrating multi-detector serial scheduling.
///
/// Owns:
///   - DetectorRegistry (multi-backend registry)
///   - VisionDetectorScheduler (round-robin pick_next)
///   - AlertEventPublisher (confirmation + cooldown)
///   - VoiceExclusiveGuard (shared with voice pipeline)
///
/// Callbacks deliver results to the ROS layer (ai_runtime_node).
///
/// Not thread-safe for public methods — call from a single worker
/// thread or under external synchronization.
class VisionPipeline
{
public:
  struct FrameInput
  {
    std::string frame_id;
    uint32_t width{0};
    uint32_t height{0};
    std::string encoding;
    std::vector<uint8_t> data;
    uint32_t step{0};
    uint64_t generation{0};
  };

  struct Detection2DOutput
  {
    std::string frame_id;
    uint64_t generation{0};
    std::string detector_source;
    std::vector<VisionBackend::Detection> detections;
  };

  struct PoseOutput
  {
    std::string frame_id;
    uint64_t generation{0};
    std::string detector_source;
    int64_t detection_id{0};
    // Pose keypoints to be populated by the Pose backend.
    struct Keypoint { int id; float x, y, confidence; };
    std::vector<Keypoint> keypoints;
    float confidence{0.0f};
  };

  using DetectionCallback =
      std::function<void(const Detection2DOutput&)>;
  using PoseCallback =
      std::function<void(const PoseOutput&)>;
  using AlertCallback =
      std::function<void(const AlertEventPublisher::AlertInfo&)>;

  VisionPipeline();

  // ── Lifecycle ──

  /// Register a detector backend with the pipeline.
  void register_detector(const std::string& name,
                         std::unique_ptr<VisionBackend> backend);

  /// Load all registered detectors.
  void load_all();

  /// Warmup all registered detectors.
  void warmup_all(const CancellationToken& token,
                  std::chrono::steady_clock::time_point deadline);

  /// Unload all detectors.
  void unload_all();

  // ── Frame processing ──

  /// Accept a new camera frame.  The pipeline stores the latest
  /// frame (atomic replacement via LatestFrameBuffer).
  void put_frame(const FrameInput& frame);

  /// Build an InferenceJob for the next detector (as chosen by
  /// VisionDetectorScheduler::pick_next).  Returns std::nullopt
  /// when no detector is due or voice has priority.
  ///
  /// The returned job is ready for submission to RuntimeScheduler.
  /// The job's execute function runs the detector backend, then
  /// calls the appropriate output callback.
  std::optional<InferenceJob> build_next_job(
      uint64_t epoch,
      const std::function<bool(uint64_t)>& epoch_is_current,
      const std::function<bool(JobModule)>& module_enabled,
      const std::function<bool()>& cancel_predicate,
      std::chrono::steady_clock::time_point now);

  // ── Voice coordination ──

  /// Shared guard — set by the node, read by the scheduler.
  VoiceExclusiveGuard& voice_guard() { return voice_guard_; }
  const VoiceExclusiveGuard& voice_guard() const { return voice_guard_; }

  // ── Profile ──

  /// Update the detector schedule from a RuntimeProfile.
  void apply_profile(const RuntimeProfile& profile);

  // ── Callbacks ──

  void set_detection_callback(DetectionCallback cb);
  void set_pose_callback(PoseCallback cb);
  void set_alert_callback(AlertCallback cb);

  // ── Accessors ──

  DetectorRegistry& registry() { return registry_; }
  VisionDetectorScheduler& scheduler() { return scheduler_; }
  AlertEventPublisher& alert_publisher() { return alert_publisher_; }
  const DetectorRegistry& registry() const { return registry_; }

private:
  DetectorRegistry registry_;
  LatestFrameBuffer<FrameInput> frame_buffer_;
  VisionDetectorScheduler scheduler_;
  AlertEventPublisher alert_publisher_;
  VoiceExclusiveGuard voice_guard_;

  DetectionCallback detection_cb_;
  PoseCallback pose_cb_;
  AlertCallback alert_cb_;

  std::mutex cb_mutex_;  // protects callbacks

  uint64_t last_job_sequence_{0};

  // Helpers
  void invoke_detection(const Detection2DOutput& out);
  void invoke_pose(const PoseOutput& out);
  void invoke_alert(const AlertEventPublisher::AlertInfo& info);

  // After inference, check for fire/fall alerts from the detections.
  void process_alerts_from_detections(
      const std::string& detector_source,
      const std::vector<VisionBackend::Detection>& detections);
  void process_alerts_from_pose(const PoseOutput& out);
};

}  // namespace k1muse_ai_runtime
