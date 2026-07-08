#include <limits>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/runtime_config.hpp"

using k1muse_ai_runtime::RuntimeConfig;

TEST(RuntimeConfig, ValidDefaults)
{
  const auto config = RuntimeConfig::defaults();
  EXPECT_TRUE(config.validate().empty());
  EXPECT_EQ(config.backend_mode, "mock");
  EXPECT_EQ(config.wakeword_provider, "cpu");
  EXPECT_EQ(config.vad_provider, "cpu");
}

TEST(RuntimeConfig, InvalidProvider)
{
  auto config = RuntimeConfig::defaults();
  config.asr_provider = "gpu";
  EXPECT_NE(config.validate().find("asr_provider"), std::string::npos);
}

TEST(RuntimeConfig, InvalidCapacity)
{
  auto config = RuntimeConfig::defaults();
  config.guarded_queue_capacity = -1;
  EXPECT_NE(config.validate().find("guarded_queue_capacity"), std::string::npos);
}

TEST(RuntimeConfig, RejectsValuesOutsideIntRangeBeforeNarrowing)
{
  EXPECT_FALSE(
    RuntimeConfig::checked_int(
      "capacity", std::numeric_limits<int64_t>::max()).has_value());
  EXPECT_FALSE(
    RuntimeConfig::checked_int(
      "capacity", std::numeric_limits<int64_t>::min()).has_value());
  EXPECT_EQ(RuntimeConfig::checked_int("capacity", 16).value(), 16);
}

TEST(RuntimeConfig, InvalidThreadCount)
{
  auto config = RuntimeConfig::defaults();
  config.spacemit_ep_intra_threads = 0;
  EXPECT_NE(config.validate().find("spacemit_ep_intra_threads"), std::string::npos);
}

TEST(RuntimeConfig, InvalidTimeout)
{
  auto config = RuntimeConfig::defaults();
  config.stop_timeout_ms = 0;
  EXPECT_NE(config.validate().find("stop_timeout_ms"), std::string::npos);
}
