#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/vision/latest_frame_buffer.hpp"

using k1muse_ai_runtime::LatestFrameBuffer;

TEST(LatestFrameBuffer, InitialEmpty)
{
  LatestFrameBuffer<int> buffer;
  EXPECT_FALSE(buffer.has_frame());
  EXPECT_EQ(buffer.get(), std::nullopt);
  EXPECT_EQ(buffer.generation(), 0U);
  EXPECT_EQ(buffer.put_count(), 0U);
  EXPECT_EQ(buffer.get_count(), 0U);
}

TEST(LatestFrameBuffer, PutAndGet)
{
  LatestFrameBuffer<int> buffer;
  buffer.put(42);
  EXPECT_TRUE(buffer.has_frame());
  auto entry = buffer.get();
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->frame, 42);
  EXPECT_EQ(entry->generation, 1U);
}

TEST(LatestFrameBuffer, GenerationIncrements)
{
  LatestFrameBuffer<int> buffer;
  buffer.put(10);
  EXPECT_EQ(buffer.generation(), 1U);
  buffer.put(20);
  EXPECT_EQ(buffer.generation(), 2U);
  buffer.put(30);
  EXPECT_EQ(buffer.generation(), 3U);
}

TEST(LatestFrameBuffer, LatestWins)
{
  LatestFrameBuffer<std::string> buffer;
  buffer.put("A");
  buffer.put("B");
  auto entry = buffer.get();
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->frame, "B");
  EXPECT_EQ(entry->generation, 2U);
}

TEST(LatestFrameBuffer, ClearWorks)
{
  LatestFrameBuffer<int> buffer;
  buffer.put(42);
  EXPECT_TRUE(buffer.has_frame());
  buffer.clear();
  EXPECT_FALSE(buffer.has_frame());
  EXPECT_EQ(buffer.get(), std::nullopt);
}

TEST(LatestFrameBuffer, ClearPreservesGeneration)
{
  LatestFrameBuffer<int> buffer;
  buffer.put(1);
  EXPECT_EQ(buffer.generation(), 1U);
  buffer.clear();
  EXPECT_EQ(buffer.generation(), 1U);
  buffer.put(2);
  EXPECT_EQ(buffer.generation(), 2U);
}

TEST(LatestFrameBuffer, PutCountTracks)
{
  LatestFrameBuffer<int> buffer;
  for (int i = 0; i < 5; ++i) {
    buffer.put(i);
  }
  EXPECT_EQ(buffer.put_count(), 5U);
}

TEST(LatestFrameBuffer, GetCountTracks)
{
  LatestFrameBuffer<int> buffer;
  buffer.put(42);
  for (int i = 0; i < 3; ++i) {
    (void)buffer.get();
  }
  EXPECT_EQ(buffer.get_count(), 3U);
}

TEST(LatestFrameBuffer, ThreadSafety)
{
  LatestFrameBuffer<int> buffer;
  constexpr int kThreads = 4;
  constexpr int kIterations = 1000;

  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kIterations; ++i) {
        buffer.put(t * kIterations + i);
      }
    });
  }
  for (auto & thread : threads) {
    thread.join();
  }

  // All puts should have succeeded; generation and put_count == kThreads * kIterations
  const uint64_t expected = static_cast<uint64_t>(kThreads) * kIterations;
  EXPECT_EQ(buffer.put_count(), expected);
  EXPECT_EQ(buffer.generation(), expected);

  // The buffer must still be valid and return a frame
  EXPECT_TRUE(buffer.has_frame());
  auto entry = buffer.get();
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->generation, expected);
}
