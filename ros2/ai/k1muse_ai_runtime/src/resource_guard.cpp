#include "k1muse_ai_runtime/resource_guard.hpp"

#include <algorithm>

namespace k1muse_ai_runtime
{

ResourceGuard::Lease::Lease(
  ResourceGuard * guard, std::unique_lock<std::mutex> lock,
  std::chrono::steady_clock::time_point acquired_at)
: guard_(guard), lock_(std::move(lock)), acquired_at_(acquired_at)
{
}

ResourceGuard::Lease::Lease(Lease && other) noexcept
: guard_(other.guard_), lock_(std::move(other.lock_)), acquired_at_(other.acquired_at_)
{
  other.guard_ = nullptr;
}

ResourceGuard::Lease & ResourceGuard::Lease::operator=(Lease && other) noexcept
{
  if (this != &other) {
    release();
    guard_ = other.guard_;
    lock_ = std::move(other.lock_);
    acquired_at_ = other.acquired_at_;
    other.guard_ = nullptr;
  }
  return *this;
}

ResourceGuard::Lease::~Lease()
{
  release();
}

void ResourceGuard::Lease::release()
{
  if (guard_ == nullptr) {
    return;
  }
  guard_->release(acquired_at_);
  guard_ = nullptr;
  if (lock_.owns_lock()) {
    lock_.unlock();
  }
}

ResourceGuard::Lease ResourceGuard::acquire(const std::string & owner)
{
  std::unique_lock<std::mutex> lock(execution_mutex_, std::defer_lock);
  if (!lock.try_lock()) {
    {
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      ++stats_.contention_count;
    }
    lock.lock();
  }
  const auto acquired_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.owner = owner;
    ++stats_.acquire_count;
  }
  return Lease(this, std::move(lock), acquired_at);
}

std::optional<ResourceGuard::Lease> ResourceGuard::try_acquire(const std::string & owner)
{
  std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    ++stats_.contention_count;
    return std::nullopt;
  }
  const auto acquired_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.owner = owner;
    ++stats_.acquire_count;
  }
  return Lease(this, std::move(lock), acquired_at);
}

ResourceGuardStats ResourceGuard::stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

void ResourceGuard::release(std::chrono::steady_clock::time_point acquired_at)
{
  const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now() - acquired_at);
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.owner.clear();
  stats_.total_hold += duration;
  stats_.max_hold = std::max(stats_.max_hold, duration);
}

}  // namespace k1muse_ai_runtime
