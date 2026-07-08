#include <algorithm>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/resource_guard.hpp"

using k1muse_ai_runtime::ResourceGuard;

TEST(ResourceGuard, RaiiReleasesAndRecordsStatistics)
{
  ResourceGuard guard;
  {
    auto lease = guard.acquire("asr");
    EXPECT_EQ(guard.stats().owner, "asr");
  }
  const auto stats = guard.stats();
  EXPECT_TRUE(stats.owner.empty());
  EXPECT_EQ(stats.acquire_count, 1U);
  EXPECT_GE(stats.total_hold.count(), 0);
}

TEST(ResourceGuard, ExceptionReleasesLease)
{
  ResourceGuard guard;
  try {
    auto lease = guard.acquire("vision");
    throw std::runtime_error("mock failure");
  } catch (const std::runtime_error &) {
  }
  EXPECT_TRUE(guard.stats().owner.empty());
  auto lease = guard.acquire("tts");
  EXPECT_EQ(guard.stats().acquire_count, 2U);
}

TEST(ResourceGuard, MutualExclusionAcrossThreads)
{
  ResourceGuard guard;
  std::atomic<int> active{0};
  std::atomic<int> maximum{0};
  std::vector<std::thread> threads;
  for (int index = 0; index < 6; ++index) {
    threads.emplace_back([&, index]() {
      auto lease = guard.acquire("worker-" + std::to_string(index));
      const int current = ++active;
      maximum.store(std::max(maximum.load(), current));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      --active;
    });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(maximum.load(), 1);
  EXPECT_GE(guard.stats().contention_count, 1U);
}
