#include "k1muse_ai_runtime/pipelines/vision_pipeline.hpp"

#include <algorithm>
#include <sstream>

namespace k1muse_ai_runtime
{

VisionPipeline::VisionPipeline()
{
  // Default: register a single YOLO detector config.
  // Real registration happens via apply_profile() or explicit
  // register_detector() calls from the node.
}

// ── Lifecycle ─────────────────────────────────────────────────

void VisionPipeline::register_detector(
    const std::string& name, std::unique_ptr<VisionBackend> backend)
{
  registry_.register_detector(name, std::move(backend));
}

void VisionPipeline::load_all()
{
  // Load is handled by individual backends through the registry.
  // The node calls backend->load() via the registry.
}

void VisionPipeline::warmup_all(
    const CancellationToken& /*token*/,
    std::chrono::steady_clock::time_point /*deadline*/)
{
  // Warmup is triggered per-detector.  The node iterates the
  // registry and calls each backend's warmup if available.
}

void VisionPipeline::unload_all()
{
  // Unload handled per-backend.
}

// ── Frame ─────────────────────────────────────────────────────

void VisionPipeline::put_frame(const FrameInput& frame)
{
  frame_buffer_.put(frame);
}

// ── Job building ──────────────────────────────────────────────

std::optional<InferenceJob> VisionPipeline::build_next_job(
    uint64_t epoch,
    const std::function<bool(uint64_t)>& epoch_is_current,
    const std::function<bool(JobModule)>& module_enabled,
    const std::function<bool()>& cancel_predicate,
    std::chrono::steady_clock::time_point now)
{
  // Voice check.
  if (voice_guard_.voice_active()) {
    return std::nullopt;
  }

  // Pick next detector.
  auto maybe_name = scheduler_.pick_next(voice_guard_, now);
  if (!maybe_name) {
    return std::nullopt;
  }

  const std::string detector_name = *maybe_name;
  VisionBackend* backend = registry_.get_detector(detector_name);
  if (!backend || !backend->loaded()) {
    scheduler_.on_completed(detector_name, now, 0, false);
    return std::nullopt;
  }

  // Get latest frame for inference.
  auto entry = frame_buffer_.get();
  if (!entry) {
    scheduler_.on_completed(detector_name, now, 0, false);
    return std::nullopt;  // No frame available yet.
  }
  FrameInput frame = std::move(entry->frame);
  uint64_t gen = entry->generation;

  // Build the job.
  InferenceJob job;
  job.id = "vision_" + detector_name + "_" + std::to_string(++last_job_sequence_);
  job.module = JobModule::VISION_EP;
  job.provider_class = ProviderClass::GuardedEp;
  job.priority = canonical_priority(JobModule::VISION_EP);
  job.epoch = epoch;
  job.deadline = now + std::chrono::milliseconds(3000);  // 3 s deadline
  job.epoch_is_current = epoch_is_current;
  job.module_enabled = module_enabled;
  job.cancel_predicate = cancel_predicate;
  job.sequence = last_job_sequence_;

  // Capture what we need for the execute lambda.
  // frame and gen are captured by value (frame may be large, but
  // we only store the latest — typically < 1 MB for 640x480 BGR).
  job.execute = [this, detector_name, backend, frame = std::move(frame),
                 gen](const CancellationToken& token) {
    voice_guard_.on_vision_started();

    auto t_start = std::chrono::steady_clock::now();

    // Run inference.
    auto result = backend->infer(
        frame.frame_id, static_cast<int>(frame.width),
        static_cast<int>(frame.height), frame.encoding,
        frame.data.data(), frame.data.size(),
        frame.step);

    auto t_end = std::chrono::steady_clock::now();
    int64_t latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        t_end - t_start).count();

    // Check if generation is still current.
    auto latest = frame_buffer_.get();
    bool stale = latest.has_value() && (latest->generation != gen);

    // Publish detections (if not stale).
    if (!stale && !token.stop_requested() && result.success) {
      Detection2DOutput out;
      out.frame_id = frame.frame_id;
      out.generation = gen;
      out.detector_source = detector_name;
      out.detections = std::move(result.detections);
      invoke_detection(out);

      // Process alerts from detections.
      process_alerts_from_detections(detector_name, out.detections);
    }

    // Signal completion.
    scheduler_.on_completed(detector_name, t_end, latency_ms,
                            result.success && !stale);
    voice_guard_.on_vision_done();
  };

  // commit_if_current: validate epoch + vision_enabled.
  job.commit_if_current = [epoch, epoch_is_current, module_enabled,
                           cancel_predicate](const CancellationToken& token) {
    if (token.stop_requested()) return false;
    if (!epoch_is_current(epoch)) return false;
    if (!module_enabled(JobModule::VISION_EP)) return false;
    if (cancel_predicate()) return false;
    return token.try_begin_commit();
  };

  scheduler_.on_submitted(detector_name, now);

  return job;
}

// ── Profile ───────────────────────────────────────────────────

void VisionPipeline::apply_profile(const RuntimeProfile& profile)
{
  scheduler_.update_schedule(profile.vision_schedule);
}

// ── Callbacks ─────────────────────────────────────────────────

void VisionPipeline::set_detection_callback(DetectionCallback cb)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  detection_cb_ = std::move(cb);
}

void VisionPipeline::set_pose_callback(PoseCallback cb)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  pose_cb_ = std::move(cb);
}

void VisionPipeline::set_alert_callback(AlertCallback cb)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  alert_cb_ = std::move(cb);
}

// ── Private helpers ───────────────────────────────────────────

void VisionPipeline::invoke_detection(const Detection2DOutput& out)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  if (detection_cb_) detection_cb_(out);
}

void VisionPipeline::invoke_pose(const PoseOutput& out)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  if (pose_cb_) pose_cb_(out);
}

void VisionPipeline::invoke_alert(const AlertEventPublisher::AlertInfo& info)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  if (alert_cb_) alert_cb_(info);
}

void VisionPipeline::process_alerts_from_detections(
    const std::string& detector_source,
    const std::vector<VisionBackend::Detection>& detections)
{
  bool has_fire = false;
  bool has_smoke = false;
  float fire_conf = 0.0f;
  float smoke_conf = 0.0f;

  for (const auto& d : detections) {
    std::string cls = d.class_name;
    std::transform(cls.begin(), cls.end(), cls.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (cls == "fire" || cls == "flame") {
      has_fire = true;
      if (d.score > fire_conf) fire_conf = d.score;
    }
    if (cls == "smoke") {
      has_smoke = true;
      if (d.score > smoke_conf) smoke_conf = d.score;
    }
  }

  if (has_fire) {
    auto info = alert_publisher_.feed(
        AlertEventPublisher::AlertType::kFire, true, fire_conf,
        detector_source);
    if (info.fired) invoke_alert(info);
  } else {
    alert_publisher_.feed(
        AlertEventPublisher::AlertType::kFire, false, 0.0f,
        detector_source);
  }

  if (has_smoke) {
    auto info = alert_publisher_.feed(
        AlertEventPublisher::AlertType::kSmoke, true, smoke_conf,
        detector_source);
    if (info.fired) invoke_alert(info);
  } else {
    alert_publisher_.feed(
        AlertEventPublisher::AlertType::kSmoke, false, 0.0f,
        detector_source);
  }
}

void VisionPipeline::process_alerts_from_pose(const PoseOutput& out)
{
  // Feed the pose confidence as a fall-detection proxy.
  // Real fall detection uses STGCN in a second stage — this is a
  // simple threshold-based placeholder for Batch 1.
  bool possible_fall = (out.confidence > 0.5f);
  auto info = alert_publisher_.feed(
      AlertEventPublisher::AlertType::kFall,
      possible_fall, out.confidence, out.detector_source);
  if (info.fired) invoke_alert(info);
}

}  // namespace k1muse_ai_runtime
