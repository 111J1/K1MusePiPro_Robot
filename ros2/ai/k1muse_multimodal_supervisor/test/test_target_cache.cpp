#include "k1muse_multimodal_supervisor/target_cache.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace k1muse_multimodal_supervisor;
using namespace std::chrono_literals;

namespace {

// Helper: build a CachedDetection with the given parameters.
CachedDetection make_det(const std::string& id, const std::string& cls,
                         float score, uint32_t x = 0, uint32_t y = 0,
                         uint32_t w = 100, uint32_t h = 100) {
  CachedDetection d;
  d.detection_id = id;
  d.class_name   = cls;
  d.score        = score;
  d.x            = x;
  d.y            = y;
  d.width        = w;
  d.height       = h;
  return d;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. FindTargetInCache
// ---------------------------------------------------------------------------
TEST(TargetCache, FindTargetInCache) {
  TargetCache cache;
  std::vector<CachedDetection> dets;
  dets.push_back(make_det("d1", "person", 0.9f));

  cache.update(640, 480, std::move(dets),
               std::chrono::steady_clock::now());

  auto r = cache.find_target("person");
  EXPECT_TRUE(r.found);
  EXPECT_EQ(r.target_id, "d1");
  EXPECT_EQ(r.target_class, "person");
  EXPECT_FLOAT_EQ(r.score, 0.9f);
}

// ---------------------------------------------------------------------------
// 2. ExpiredDetection
// ---------------------------------------------------------------------------
TEST(TargetCache, ExpiredDetection) {
  // Use a very short TTL so the frame expires quickly.
  TargetCache cache(1ms);
  std::vector<CachedDetection> dets;
  dets.push_back(make_det("d1", "person", 0.9f));

  auto past = std::chrono::steady_clock::now() - 100ms;
  cache.update(640, 480, std::move(dets), past);

  auto r = cache.find_target("person");
  EXPECT_FALSE(r.found);
  EXPECT_EQ(r.reason, "expired");
}

// ---------------------------------------------------------------------------
// 3. NoMatchingClass
// ---------------------------------------------------------------------------
TEST(TargetCache, NoMatchingClass) {
  TargetCache cache;
  std::vector<CachedDetection> dets;
  dets.push_back(make_det("d1", "car", 0.85f));

  cache.update(640, 480, std::move(dets),
               std::chrono::steady_clock::now());

  auto r = cache.find_target("person");
  EXPECT_FALSE(r.found);
  EXPECT_EQ(r.reason, "no matching class");
}

// ---------------------------------------------------------------------------
// 4. HighestScoreWins
// ---------------------------------------------------------------------------
TEST(TargetCache, HighestScoreWins) {
  TargetCache cache;
  std::vector<CachedDetection> dets;
  dets.push_back(make_det("d1", "person", 0.7f));
  dets.push_back(make_det("d2", "person", 0.95f));
  dets.push_back(make_det("d3", "person", 0.6f));

  cache.update(640, 480, std::move(dets),
               std::chrono::steady_clock::now());

  auto r = cache.find_target("person");
  EXPECT_TRUE(r.found);
  EXPECT_EQ(r.target_id, "d2");
  EXPECT_FLOAT_EQ(r.score, 0.95f);
}

// ---------------------------------------------------------------------------
// 5. MinimumScoreFilter
// ---------------------------------------------------------------------------
TEST(TargetCache, MinimumScoreFilter) {
  TargetCache cache;
  std::vector<CachedDetection> dets;
  dets.push_back(make_det("d1", "person", 0.3f));

  cache.update(640, 480, std::move(dets),
               std::chrono::steady_clock::now());

  auto r = cache.find_target("person", 0.5f);
  EXPECT_FALSE(r.found);
}

// ---------------------------------------------------------------------------
// 6. EmptyCache
// ---------------------------------------------------------------------------
TEST(TargetCache, EmptyCache) {
  TargetCache cache;

  auto r = cache.find_target("person");
  EXPECT_FALSE(r.found);
  EXPECT_EQ(r.reason, "no frame");
}

// ---------------------------------------------------------------------------
// 7. ClearWorks
// ---------------------------------------------------------------------------
TEST(TargetCache, ClearWorks) {
  TargetCache cache;
  std::vector<CachedDetection> dets;
  dets.push_back(make_det("d1", "person", 0.9f));

  cache.update(640, 480, std::move(dets),
               std::chrono::steady_clock::now());
  EXPECT_TRUE(cache.has_valid_frame());

  cache.clear();
  EXPECT_FALSE(cache.has_valid_frame());

  auto r = cache.find_target("person");
  EXPECT_FALSE(r.found);
  EXPECT_EQ(r.reason, "no frame");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
