#include <gtest/gtest.h>

#include <sstream>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "k1muse_common/id_utils.hpp"

TEST(IdUtils, UsesPrefixAndProducesUniqueIdsAcrossThreads)
{
  constexpr std::size_t kThreadCount = 8;
  constexpr std::size_t kIdsPerThread = 64;
  std::vector<std::vector<std::string>> generated(kThreadCount);
  std::vector<std::thread> threads;

  for (std::size_t index = 0; index < kThreadCount; ++index) {
    threads.emplace_back([index, &generated]() {
      for (std::size_t count = 0; count < kIdsPerThread; ++count) {
        generated[index].push_back(k1muse_common::make_id("trace"));
      }
    });
  }
  for (auto & thread : threads) {
    thread.join();
  }

  std::set<std::string> unique;
  for (const auto & group : generated) {
    for (const auto & id : group) {
      EXPECT_EQ(id.rfind("trace-", 0), 0U);
      unique.insert(id);
    }
  }
  EXPECT_EQ(unique.size(), kThreadCount * kIdsPerThread);
}

TEST(IdUtils, IncludesCurrentProcessIdentity)
{
  const auto id = k1muse_common::make_id("request");
  std::ostringstream process_id;
  process_id << std::hex << k1muse_common::process_id();

  std::vector<std::string> fields;
  std::istringstream stream(id);
  std::string field;
  while (std::getline(stream, field, '-')) {
    fields.push_back(field);
  }

  ASSERT_EQ(fields.size(), 6U);
  EXPECT_EQ(fields[0], "request");
  EXPECT_EQ(fields[3], process_id.str());
}
