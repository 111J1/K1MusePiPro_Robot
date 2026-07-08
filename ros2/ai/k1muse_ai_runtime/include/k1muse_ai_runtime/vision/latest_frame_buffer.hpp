#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

namespace k1muse_ai_runtime
{

template<typename T>
class LatestFrameBuffer
{
public:
  struct Entry
  {
    T frame;
    uint64_t generation;
  };

  LatestFrameBuffer() = default;

  LatestFrameBuffer(const LatestFrameBuffer &) = delete;
  LatestFrameBuffer & operator=(const LatestFrameBuffer &) = delete;

  // Atomically replace the latest frame. Increments generation.
  void put(T frame);

  // Get the latest frame and its generation. Returns std::nullopt if empty.
  std::optional<Entry> get() const;

  // Get just the current generation (0 if no frame ever put).
  uint64_t generation() const;

  // Check if a frame is available.
  bool has_frame() const;

  // Clear the buffer.
  void clear();

  // Get count of frames put (total, not current).
  uint64_t put_count() const;

  // Get count of frames got (total, not current).
  uint64_t get_count() const;

private:
  mutable std::mutex mutex_;
  std::optional<T> frame_;
  uint64_t generation_{0};
  uint64_t put_count_{0};
  mutable uint64_t get_count_{0};
};

template<typename T>
void LatestFrameBuffer<T>::put(T frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  frame_ = std::move(frame);
  ++put_count_;
}

template<typename T>
std::optional<typename LatestFrameBuffer<T>::Entry> LatestFrameBuffer<T>::get() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!frame_.has_value()) {
    return std::nullopt;
  }
  ++get_count_;
  return Entry{*frame_, generation_};
}

template<typename T>
uint64_t LatestFrameBuffer<T>::generation() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return generation_;
}

template<typename T>
bool LatestFrameBuffer<T>::has_frame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_.has_value();
}

template<typename T>
void LatestFrameBuffer<T>::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  frame_ = std::nullopt;
}

template<typename T>
uint64_t LatestFrameBuffer<T>::put_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return put_count_;
}

template<typename T>
uint64_t LatestFrameBuffer<T>::get_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return get_count_;
}

}  // namespace k1muse_ai_runtime
