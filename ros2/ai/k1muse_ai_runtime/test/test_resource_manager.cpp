#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "k1muse_ai_runtime/resource_manager.hpp"

namespace k1muse_ai_runtime
{
namespace
{

// Simple mock ModelRuntime for testing
class TestModelRuntime final : public ModelRuntime
{
public:
  explicit TestModelRuntime(std::string name) : name_(std::move(name)) {}

  const std::string & name() const override { return name_; }
  const std::string & provider() const override { return provider_; }

  void load(const CancellationToken & token, Deadline deadline) override
  {
    (void)token;
    (void)deadline;
    loaded_ = true;
  }

  void warmup(const CancellationToken & token, Deadline deadline) override
  {
    (void)token;
    (void)deadline;
  }

  void request_cancel() noexcept override {}
  bool stop(std::chrono::milliseconds) noexcept override { return true; }
  void final_join() noexcept override {}
  void unload() noexcept override { loaded_ = false; }
  bool loaded() const noexcept override { return loaded_; }

private:
  std::string name_;
  std::string provider_{"cpu"};
  bool loaded_{false};
};

TEST(ResourceManager, EmptyOnCreation)
{
  ResourceManager rm;
  EXPECT_EQ(rm.size(), 0u);
  EXPECT_EQ(rm.loaded_count(), 0u);
  EXPECT_TRUE(rm.registered_models().empty());
}

TEST(ResourceManager, RegisterModel)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));

  EXPECT_EQ(rm.size(), 1u);
  EXPECT_FALSE(rm.is_loaded("wakeword"));
}

TEST(ResourceManager, RegisterMultipleModels)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));
  rm.register_model("vad_asr", std::make_unique<TestModelRuntime>("vad_asr"));
  rm.register_model("vision", std::make_unique<TestModelRuntime>("vision"));

  EXPECT_EQ(rm.size(), 3u);

  auto names = rm.registered_models();
  EXPECT_EQ(names.size(), 3u);
}

TEST(ResourceManager, LoadModel)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));

  rm.load_model("wakeword");
  EXPECT_TRUE(rm.is_loaded("wakeword"));
  EXPECT_EQ(rm.loaded_count(), 1u);
}

TEST(ResourceManager, LoadModelNotFound)
{
  ResourceManager rm;
  EXPECT_THROW(rm.load_model("nonexistent"), std::runtime_error);
}

TEST(ResourceManager, LoadModelAlreadyLoaded)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));

  rm.load_model("wakeword");
  EXPECT_THROW(rm.load_model("wakeword"), std::runtime_error);
}

TEST(ResourceManager, UnloadModel)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));

  rm.load_model("wakeword");
  EXPECT_TRUE(rm.is_loaded("wakeword"));

  rm.unload_model("wakeword");
  EXPECT_FALSE(rm.is_loaded("wakeword"));
  EXPECT_EQ(rm.loaded_count(), 0u);
}

TEST(ResourceManager, UnloadModelNotFound)
{
  ResourceManager rm;
  EXPECT_THROW(rm.unload_model("nonexistent"), std::runtime_error);
}

TEST(ResourceManager, UnloadModelAlreadyUnloaded)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));

  // Unloading an unloaded model should not throw
  rm.unload_model("wakeword");
  EXPECT_FALSE(rm.is_loaded("wakeword"));
}

TEST(ResourceManager, GetModel)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));

  auto * model = rm.get_model("wakeword");
  ASSERT_NE(model, nullptr);
  EXPECT_EQ(model->name(), "wakeword");
}

TEST(ResourceManager, GetModelNotFound)
{
  ResourceManager rm;
  EXPECT_EQ(rm.get_model("nonexistent"), nullptr);
}

TEST(ResourceManager, UnregisterModel)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));

  EXPECT_TRUE(rm.unregister_model("wakeword"));
  EXPECT_EQ(rm.size(), 0u);
  EXPECT_FALSE(rm.is_loaded("wakeword"));
}

TEST(ResourceManager, UnregisterNotFound)
{
  ResourceManager rm;
  EXPECT_FALSE(rm.unregister_model("nonexistent"));
}

TEST(ResourceManager, LoadedModels)
{
  ResourceManager rm;
  rm.register_model("wakeword", std::make_unique<TestModelRuntime>("wakeword"));
  rm.register_model("vad_asr", std::make_unique<TestModelRuntime>("vad_asr"));
  rm.register_model("vision", std::make_unique<TestModelRuntime>("vision"));

  rm.load_model("wakeword");
  rm.load_model("vision");

  auto loaded = rm.loaded_models();
  EXPECT_EQ(loaded.size(), 2u);
  // Check both are in the list
  bool found_wakeword = false;
  bool found_vision = false;
  for (const auto & name : loaded) {
    if (name == "wakeword") found_wakeword = true;
    if (name == "vision") found_vision = true;
  }
  EXPECT_TRUE(found_wakeword);
  EXPECT_TRUE(found_vision);
}

TEST(ResourceManager, MemoryUsage)
{
  ResourceManager rm;
  float usage = rm.memory_usage_mb();
  // On Linux, should return a positive value
  // On other platforms, returns 0.0
#ifdef __linux__
  EXPECT_GT(usage, 0.0f);
#else
  EXPECT_EQ(usage, 0.0f);
#endif
}

TEST(ResourceManager, TotalMemory)
{
  ResourceManager rm;
  float total = rm.total_memory_mb();
  // On Linux, should return a positive value
  // On other platforms, returns 0.0
#ifdef __linux__
  EXPECT_GT(total, 0.0f);
#else
  EXPECT_EQ(total, 0.0f);
#endif
}

TEST(ResourceManager, MemoryAboveThreshold)
{
  ResourceManager rm;
  // Should not be above a very high threshold
  EXPECT_FALSE(rm.memory_above_threshold(1000000.0f));
  // Should be above zero threshold (on Linux)
#ifdef __linux__
  EXPECT_TRUE(rm.memory_above_threshold(0.0f));
#endif
}

}  // namespace
}  // namespace k1muse_ai_runtime
