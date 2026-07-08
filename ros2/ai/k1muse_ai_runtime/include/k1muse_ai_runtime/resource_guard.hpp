#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace k1muse_ai_runtime
{

struct ResourceGuardStats
{
  std::string owner;
  uint64_t acquire_count{0};
  uint64_t contention_count{0};
  std::chrono::nanoseconds total_hold{0};
  std::chrono::nanoseconds max_hold{0};
};

class ResourceGuard
{
public:
  class Lease
  {
  public:
    Lease(const Lease &) = delete;
    Lease & operator=(const Lease &) = delete;
    Lease(Lease && other) noexcept;
    Lease & operator=(Lease && other) noexcept;
    ~Lease();

  private:
    friend class ResourceGuard;
    Lease(
      ResourceGuard * guard, std::unique_lock<std::mutex> lock,
      std::chrono::steady_clock::time_point acquired_at);
    void release();

    ResourceGuard * guard_{nullptr};
    std::unique_lock<std::mutex> lock_;
    std::chrono::steady_clock::time_point acquired_at_;
  };

  Lease acquire(const std::string & owner);
  std::optional<Lease> try_acquire(const std::string & owner);
  ResourceGuardStats stats() const;

private:
  void release(std::chrono::steady_clock::time_point acquired_at);

  mutable std::mutex execution_mutex_;
  mutable std::mutex stats_mutex_;
  ResourceGuardStats stats_;
};

}  // namespace k1muse_ai_runtime
