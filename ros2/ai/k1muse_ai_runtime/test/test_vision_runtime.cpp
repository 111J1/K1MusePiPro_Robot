#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/backends/mock_vision_backend.hpp"
#include "k1muse_ai_runtime/inference_job.hpp"
#include "k1muse_ai_runtime/models/vision_runtime.hpp"

using k1muse_ai_runtime::CancellationToken;
using k1muse_ai_runtime::JobModule;
using k1muse_ai_runtime::MockVisionBackend;
using k1muse_ai_runtime::VisionBackend;
using k1muse_ai_runtime::VisionRuntime;

namespace
{

struct VisionResult
{
  int call_count{0};
  std::string trace_id;
  uint64_t epoch{0};
  uint32_t image_width{0};
  uint32_t image_height{0};
  std::vector<VisionRuntime::DetectionData> detections;
};

std::unique_ptr<VisionRuntime> make_vision_runtime(
  VisionResult & result, const std::string & provider = "cpu")
{
  auto backend = std::make_unique<MockVisionBackend>();
  auto callback = [&result](
    const std::string & trace_id, uint64_t epoch,
    uint32_t image_width, uint32_t image_height,
    std::vector<VisionRuntime::DetectionData> detections) {
      ++result.call_count;
      result.trace_id = trace_id;
      result.epoch = epoch;
      result.image_width = image_width;
      result.image_height = image_height;
      result.detections = std::move(detections);
    };
  return std::make_unique<VisionRuntime>(
    std::move(backend), provider, std::move(callback));
}

auto epoch_true = [](uint64_t) { return true; };
auto module_enabled_true = [](JobModule) { return true; };
auto cancel_false = []() { return false; };

constexpr uint32_t kTestWidth = 640;
constexpr uint32_t kTestHeight = 480;

}  // namespace

// --- Test 1: PutAndProcess ---
// put_frame + process_latest_frame -> result callback fires.
TEST(VisionRuntimeTest, PutAndProcess)
{
  VisionResult result;
  auto runtime = make_vision_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  runtime->put_frame("frame_1", kTestWidth, kTestHeight, "rgb8", "trace_1", 1);

  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_latest_frame(
    epoch_true, module_enabled_true, cancel_false, now + std::chrono::seconds(1), 1);

  EXPECT_TRUE(processed);
  EXPECT_EQ(result.call_count, 1);
  EXPECT_EQ(result.trace_id, "trace_1");
  EXPECT_EQ(result.epoch, 1U);
  EXPECT_EQ(result.image_width, kTestWidth);
  EXPECT_EQ(result.image_height, kTestHeight);
  EXPECT_EQ(result.detections.size(), 1U);

  runtime->unload();
}

// --- Test 2: LatestFrameWins ---
// put A, put B, process -> only B processed.
TEST(VisionRuntimeTest, LatestFrameWins)
{
  VisionResult result;
  auto runtime = make_vision_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  runtime->put_frame("frame_A", kTestWidth, kTestHeight, "rgb8", "trace_A", 1);
  runtime->put_frame("frame_B", kTestWidth, kTestHeight, "rgb8", "trace_B", 2);

  auto now = std::chrono::steady_clock::now();
  runtime->process_latest_frame(
    epoch_true, module_enabled_true, cancel_false, now + std::chrono::seconds(1), 2);

  EXPECT_EQ(result.call_count, 1);
  EXPECT_EQ(result.trace_id, "trace_B");

  runtime->unload();
}

// --- Test 3: NoFrameReturnsFalse ---
// process with empty buffer -> returns false.
TEST(VisionRuntimeTest, NoFrameReturnsFalse)
{
  VisionResult result;
  auto runtime = make_vision_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_latest_frame(
    epoch_true, module_enabled_true, cancel_false, now + std::chrono::seconds(1), 1);

  EXPECT_FALSE(processed);
  EXPECT_EQ(result.call_count, 0);

  runtime->unload();
}

// --- Test 4: StopPreventsProcessing ---
// stop(), process -> returns false, no callback.
TEST(VisionRuntimeTest, StopPreventsProcessing)
{
  VisionResult result;
  auto runtime = make_vision_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  runtime->put_frame("frame_1", kTestWidth, kTestHeight, "rgb8", "trace_1", 1);
  runtime->stop(std::chrono::milliseconds(100));

  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_latest_frame(
    epoch_true, module_enabled_true, cancel_false, now + std::chrono::seconds(1), 1);

  EXPECT_FALSE(processed);
  EXPECT_EQ(result.call_count, 0);

  runtime->unload();
}

// --- Test 5: BackendFailureNoCallback ---
// backend returns success=false -> no callback.
TEST(VisionRuntimeTest, BackendFailureNoCallback)
{
  VisionResult result;
  auto runtime = make_vision_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  // Zero dimensions cause MockVisionBackend to return success=false.
  runtime->put_frame("frame_1", 0, 0, "rgb8", "trace_1", 1);

  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_latest_frame(
    epoch_true, module_enabled_true, cancel_false, now + std::chrono::seconds(1), 1);

  // Frame was processed (generation match) but backend failed.
  EXPECT_TRUE(processed);
  EXPECT_EQ(result.call_count, 0);

  runtime->unload();
}

// --- Test 6: ClearBufferWorks ---
// put, clear, process -> returns false.
TEST(VisionRuntimeTest, ClearBufferWorks)
{
  VisionResult result;
  auto runtime = make_vision_runtime(result);

  CancellationToken token;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  runtime->load(token, deadline);

  runtime->put_frame("frame_1", kTestWidth, kTestHeight, "rgb8", "trace_1", 1);
  runtime->clear_buffer();

  auto now = std::chrono::steady_clock::now();
  bool processed = runtime->process_latest_frame(
    epoch_true, module_enabled_true, cancel_false, now + std::chrono::seconds(1), 1);

  EXPECT_FALSE(processed);
  EXPECT_EQ(result.call_count, 0);

  runtime->unload();
}
