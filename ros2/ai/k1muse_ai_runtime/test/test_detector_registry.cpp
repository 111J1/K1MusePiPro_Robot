#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "k1muse_ai_runtime/detector_registry.hpp"

namespace k1muse_ai_runtime
{
namespace
{

// Simple mock VisionBackend for testing
class TestVisionBackend final : public VisionBackend
{
public:
  explicit TestVisionBackend(std::string name) : name_(std::move(name)) {}

  const std::string & name() const override { return name_; }
  void load() override { loaded_ = true; }
  void unload() override { loaded_ = false; }
  bool loaded() const override { return loaded_; }
  void warmup() override {}

  FrameResult infer(
    const std::string & frame_id, uint32_t width, uint32_t height,
    const std::string & encoding, const uint8_t * data, size_t data_size,
    uint32_t step) override
  {
    (void)frame_id;
    (void)data;
    (void)data_size;
    (void)step;
    (void)width;
    (void)height;
    (void)encoding;
    (void)data;
    (void)data_size;
    FrameResult result;
    result.success = true;
    return result;
  }

  void reset() override {}

private:
  std::string name_;
  bool loaded_{false};
};

TEST(DetectorRegistry, EmptyOnCreation)
{
  DetectorRegistry registry;
  EXPECT_EQ(registry.size(), 0u);
  EXPECT_TRUE(registry.registered_detectors().empty());
}

TEST(DetectorRegistry, RegisterDetector)
{
  DetectorRegistry registry;
  registry.register_detector("yolov8n", std::make_unique<TestVisionBackend>("yolov8n"));

  EXPECT_EQ(registry.size(), 1u);
  EXPECT_TRUE(registry.has_detector("yolov8n"));
  EXPECT_FALSE(registry.has_detector("pose"));
}

TEST(DetectorRegistry, RegisterMultipleDetectors)
{
  DetectorRegistry registry;
  registry.register_detector("yolov8n", std::make_unique<TestVisionBackend>("yolov8n"));
  registry.register_detector("pose", std::make_unique<TestVisionBackend>("pose"));
  registry.register_detector("fire", std::make_unique<TestVisionBackend>("fire"));

  EXPECT_EQ(registry.size(), 3u);

  auto names = registry.registered_detectors();
  EXPECT_EQ(names.size(), 3u);
  // Names are sorted in map order
  EXPECT_EQ(names[0], "fire");
  EXPECT_EQ(names[1], "pose");
  EXPECT_EQ(names[2], "yolov8n");
}

TEST(DetectorRegistry, GetDetector)
{
  DetectorRegistry registry;
  registry.register_detector("yolov8n", std::make_unique<TestVisionBackend>("yolov8n"));

  auto * backend = registry.get_detector("yolov8n");
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "yolov8n");
}

TEST(DetectorRegistry, GetDetectorNotFound)
{
  DetectorRegistry registry;
  EXPECT_EQ(registry.get_detector("nonexistent"), nullptr);
}

TEST(DetectorRegistry, UnregisterDetector)
{
  DetectorRegistry registry;
  registry.register_detector("yolov8n", std::make_unique<TestVisionBackend>("yolov8n"));

  EXPECT_TRUE(registry.unregister_detector("yolov8n"));
  EXPECT_EQ(registry.size(), 0u);
  EXPECT_FALSE(registry.has_detector("yolov8n"));
}

TEST(DetectorRegistry, UnregisterNotFound)
{
  DetectorRegistry registry;
  EXPECT_FALSE(registry.unregister_detector("nonexistent"));
}

TEST(DetectorRegistry, ReplaceDetector)
{
  DetectorRegistry registry;
  registry.register_detector("yolov8n", std::make_unique<TestVisionBackend>("v1"));
  registry.register_detector("yolov8n", std::make_unique<TestVisionBackend>("v2"));

  EXPECT_EQ(registry.size(), 1u);
  auto * backend = registry.get_detector("yolov8n");
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "v2");
}

}  // namespace
}  // namespace k1muse_ai_runtime
