#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace k1muse_common
{

inline std::uint64_t process_id()
{
#ifdef _WIN32
  return static_cast<std::uint64_t>(::_getpid());
#else
  return static_cast<std::uint64_t>(::getpid());
#endif
}

inline std::string make_id(const std::string & prefix = "id")
{
  static std::atomic<std::uint64_t> sequence{0};
  const auto wall_time = std::chrono::system_clock::now().time_since_epoch().count();
  const auto monotonic_time = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto thread_hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  const auto value = sequence.fetch_add(1, std::memory_order_relaxed);

  std::ostringstream stream;
  if (!prefix.empty()) {
    stream << prefix << '-';
  }
  stream << std::hex << wall_time << '-' << monotonic_time << '-' << process_id() << '-' <<
    thread_hash << '-' << value;
  return stream.str();
}

}  // namespace k1muse_common
