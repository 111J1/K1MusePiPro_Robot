#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "k1muse_ai_runtime/backends/mock_fire_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_pose_backend.hpp"
#include "k1muse_ai_runtime/backends/mock_vision_backend.hpp"
#include "k1muse_ai_runtime/pipelines/vision_pipeline.hpp"
#include "k1muse_ai_runtime/runtime_profile_manager.hpp"

namespace k1muse_ai_runtime
{
namespace
{

using namespace std::chrono_literals;

// ── Helpers ──

auto NeverExpires() {
  return std::chrono::steady_clock::now() + 60s;
}

auto AlwaysCurrent(uint64_t) -> bool { return true; }
auto AlwaysEnabled(JobModule) -> bool { return true; }
auto NeverCancel() -> bool { return false; }

VisionPipeline::FrameInput MakeFrame(uint64_t gen)
{
  VisionPipeline::FrameInput f;
  f.frame_id = "frame_" + std::to_string(gen);
  f.width = 640;
  f.height = 480;
  f.encoding = "bgr8";
  f.data.resize(640 * 480 * 3, 128);
  f.generation = gen;
  return f;
}

// ── Test 1: Empty registry returns no job ──

TEST(VisionPipelineTest, EmptyRegistryReturnsNullopt)
{
  VisionPipeline pipeline;
  pipeline.put_frame(MakeFrame(1));

  auto job = pipeline.build_next_job(
      1, AlwaysCurrent, AlwaysEnabled, NeverCancel,
      std::chrono::steady_clock::now());

  EXPECT_FALSE(job.has_value());
}

// ── Test 2: Single detector builds a job ──

TEST(VisionPipelineTest, SingleDetectorBuildsJob)
{
  VisionPipeline pipeline;

  auto backend = std::make_unique<MockVisionBackend>();
  backend->set_mock_detection_count(1);
  backend->set_mock_detection_class("person");
  backend->load();
  pipeline.register_detector("yolov8n", std::move(backend));

  // Apply NORMAL profile (single YOLO detector).
  RuntimeProfile profile;
  profile.mode = RuntimeProfile::Mode::NORMAL;
  profile.vision_schedule = {
    {"yolov8n", 0, 0, true, true, "vision"},
  };
  pipeline.apply_profile(profile);

  pipeline.put_frame(MakeFrame(1));

  auto job = pipeline.build_next_job(
      1, AlwaysCurrent, AlwaysEnabled, NeverCancel,
      std::chrono::steady_clock::now());

  EXPECT_TRUE(job.has_value());
  EXPECT_NE(job->id.find("vision_yolov8n"), std::string::npos);
  EXPECT_EQ(job->module, JobModule::VISION_EP);

  // Execute the job.
  int detections_received = 0;
  pipeline.set_detection_callback(
      [&](const VisionPipeline::Detection2DOutput& out) {
        detections_received += static_cast<int>(out.detections.size());
      });

  CancellationToken token;
  job->execute(token);

  EXPECT_EQ(detections_received, 1);
}

// ── Test 3: Voice priority blocks vision ──

TEST(VisionPipelineTest, VoicePriorityBlocksVision)
{
  VisionPipeline pipeline;

  auto backend = std::make_unique<MockVisionBackend>();
  backend->load();
  pipeline.register_detector("yolov8n", std::move(backend));

  RuntimeProfile profile;
  profile.mode = RuntimeProfile::Mode::NORMAL;
  profile.vision_schedule = {{"yolov8n", 0, 0, true, true, "vision"}};
  pipeline.apply_profile(profile);

  pipeline.put_frame(MakeFrame(1));

  // Voice requests EP.
  pipeline.voice_guard().request_voice();

  auto job = pipeline.build_next_job(
      1, AlwaysCurrent, AlwaysEnabled, NeverCancel,
      std::chrono::steady_clock::now());

  // Voice active + no running vision → should be nullopt.
  EXPECT_FALSE(job.has_value());
  EXPECT_TRUE(pipeline.voice_guard().voice_active());

  // Release voice.
  pipeline.voice_guard().release_voice();
  EXPECT_FALSE(pipeline.voice_guard().voice_active());

  // Now vision can submit.
  auto job2 = pipeline.build_next_job(
      1, AlwaysCurrent, AlwaysEnabled, NeverCancel,
      std::chrono::steady_clock::now());
  EXPECT_TRUE(job2.has_value());
}

// ── Test 4: Round-robin between Pose and Fire ──

TEST(VisionPipelineTest, RoundRobinPoseFire)
{
  VisionPipeline pipeline;

  auto pose = std::make_unique<MockPoseBackend>();
  pose->load();
  pipeline.register_detector("yolov8n-pose", std::move(pose));

  auto fire = std::make_unique<MockFireBackend>();
  fire->load();
  fire->set_fire_present(true);
  pipeline.register_detector("yolov8_fire", std::move(fire));

  // GUARD profile: Pose + Fire.
  RuntimeProfile profile;
  profile.mode = RuntimeProfile::Mode::GUARD;
  profile.vision_schedule = {
    {"yolov8n-pose", 0, 0, true, true, "vision"},
    {"yolov8_fire",  0, 0, true, true, "vision"},
  };
  pipeline.apply_profile(profile);

  pipeline.put_frame(MakeFrame(1));
  auto now = std::chrono::steady_clock::now();

  // First pick → pose.
  auto job1 = pipeline.build_next_job(1, AlwaysCurrent, AlwaysEnabled,
                                      NeverCancel, now);
  ASSERT_TRUE(job1.has_value());
  EXPECT_NE(job1->id.find("yolov8n-pose"), std::string::npos);

  // Execute job1.
  CancellationToken token;
  job1->execute(token);

  // Second pick → fire.
  auto job2 = pipeline.build_next_job(1, AlwaysCurrent, AlwaysEnabled,
                                      NeverCancel, now);
  ASSERT_TRUE(job2.has_value());
  EXPECT_NE(job2->id.find("yolov8_fire"), std::string::npos);

  job2->execute(token);

  // Third pick → back to pose (round-robin).
  auto job3 = pipeline.build_next_job(1, AlwaysCurrent, AlwaysEnabled,
                                      NeverCancel, now);
  ASSERT_TRUE(job3.has_value());
  EXPECT_NE(job3->id.find("yolov8n-pose"), std::string::npos);
}

// ── Test 5: AlertPublisher fires after N consecutive positives ──

TEST(VisionPipelineTest, AlertPublisherFiresAfterConfirmation)
{
  VisionPipeline pipeline;

  auto fire = std::make_unique<MockFireBackend>();
  fire->load();
  fire->set_fire_present(true);
  pipeline.register_detector("yolov8_fire", std::move(fire));

  RuntimeProfile profile;
  profile.mode = RuntimeProfile::Mode::GUARD;
  profile.vision_schedule = {{"yolov8_fire", 0, 0, true, true, "vision"}};
  pipeline.apply_profile(profile);

  int alert_count = 0;
  pipeline.set_alert_callback(
      [&](const AlertEventPublisher::AlertInfo&) { ++alert_count; });

  auto now = std::chrono::steady_clock::now();

  // Fire needs 3 consecutive positives to fire.
  for (int i = 0; i < 3; ++i) {
    pipeline.put_frame(MakeFrame(static_cast<uint64_t>(i + 1)));
    auto job = pipeline.build_next_job(1, AlwaysCurrent, AlwaysEnabled,
                                       NeverCancel, now);
    ASSERT_TRUE(job.has_value());
    CancellationToken token;
    job->execute(token);
  }

  // After 3 consecutive fire detections → alert should fire once.
  EXPECT_EQ(alert_count, 1);

  // Next detection should NOT fire again (cooldown).
  pipeline.put_frame(MakeFrame(4));
  auto job4 = pipeline.build_next_job(1, AlwaysCurrent, AlwaysEnabled,
                                      NeverCancel, now);
  ASSERT_TRUE(job4.has_value());
  CancellationToken token4;
  job4->execute(token4);

  EXPECT_EQ(alert_count, 1); // Still 1, in cooldown.
}

// ── Test 6: RuntimeProfileManager apply/ack flow ──

TEST(RuntimeProfileManagerTest, ApplyAckFlow)
{
  RuntimeProfileManager mgr;
  EXPECT_EQ(mgr.current_mode(), RuntimeProfile::Mode::NORMAL);
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::IDLE);

  // Request GUARD mode.
  bool ok = mgr.request_profile(RuntimeProfile::Mode::GUARD);
  EXPECT_TRUE(ok);
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::REQUESTED);

  // Advance through stages.
  mgr.advance(); // REQUESTED → APPLYING
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::APPLYING);

  mgr.advance(); // APPLYING → DRAINING
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::DRAINING);

  // Signal drain complete.
  mgr.on_drain_complete();
  mgr.advance(); // DRAINING → WARMING
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::WARMING);

  // Signal warmup complete.
  mgr.on_warmup_complete();
  mgr.advance(); // WARMING → COMMITTING
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::COMMITTING);

  mgr.advance(); // COMMITTING → APPLIED
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::APPLIED);
  EXPECT_EQ(mgr.current_mode(), RuntimeProfile::Mode::GUARD);

  // Guard profile should have two detectors.
  EXPECT_EQ(mgr.current_profile().vision_schedule.size(), 2u);
}

// ── Test 7: RuntimeProfileManager rollback on failure ──

TEST(RuntimeProfileManagerTest, RollbackOnFailure)
{
  RuntimeProfileManager mgr;
  EXPECT_EQ(mgr.current_mode(), RuntimeProfile::Mode::NORMAL);

  mgr.request_profile(RuntimeProfile::Mode::GUARD);
  mgr.advance(); // REQUESTED → APPLYING
  mgr.advance(); // APPLYING → DRAINING

  // Fail during draining.
  mgr.on_failure("detector warmup timeout");

  // Should roll back to NORMAL.
  EXPECT_EQ(mgr.current_mode(), RuntimeProfile::Mode::NORMAL);
  EXPECT_EQ(mgr.apply_state(), RuntimeProfileManager::ApplyState::APPLIED);
  EXPECT_NE(mgr.last_error().find("timeout"), std::string::npos);
}

// ── Test 8: VoiceExclusiveGuard drain semantics ──

TEST(VoiceExclusiveGuardTest, DrainSemantics)
{
  VoiceExclusiveGuard guard;

  // Initial state.
  EXPECT_FALSE(guard.voice_active());
  EXPECT_TRUE(guard.can_submit_vision());
  EXPECT_EQ(guard.vision_in_flight(), 0);

  // Vision job starts.
  guard.on_vision_started();
  EXPECT_EQ(guard.vision_in_flight(), 1);
  EXPECT_TRUE(guard.can_submit_vision());

  // Voice arrives.
  guard.request_voice();
  EXPECT_FALSE(guard.can_submit_vision()); // blocked
  EXPECT_FALSE(guard.voice_active());      // vision still running

  // Vision finishes.
  guard.on_vision_done();
  EXPECT_EQ(guard.vision_in_flight(), 0);
  EXPECT_TRUE(guard.voice_active());       // voice now active

  // Voice releases.
  guard.release_voice();
  EXPECT_FALSE(guard.voice_active());
  EXPECT_TRUE(guard.can_submit_vision());
}

// ── Test 9: AlertEventPublisher reset on negatives ──

TEST(AlertEventPublisherTest, ResetOnNegatives)
{
  AlertEventPublisher::Config cfg;
  cfg.fire_confirm_frames = 3;
  AlertEventPublisher pub(cfg);

  // Feed 2 positives.
  pub.feed(AlertEventPublisher::AlertType::kFire, true, 0.9f, "test");
  pub.feed(AlertEventPublisher::AlertType::kFire, true, 0.9f, "test");

  // Feed 3 negatives → window resets.
  for (int i = 0; i < 3; ++i) {
    auto info = pub.feed(AlertEventPublisher::AlertType::kFire, false, 0.0f,
                         "test");
    EXPECT_FALSE(info.fired);
  }

  // Now feed 3 positives again — should fire once.
  for (int i = 0; i < 2; ++i) {
    pub.feed(AlertEventPublisher::AlertType::kFire, true, 0.9f, "test");
  }
  auto info = pub.feed(AlertEventPublisher::AlertType::kFire, true, 0.9f,
                       "test");
  EXPECT_TRUE(info.fired);
  EXPECT_EQ(info.type, AlertEventPublisher::AlertType::kFire);
}

}  // namespace
}  // namespace k1muse_ai_runtime
