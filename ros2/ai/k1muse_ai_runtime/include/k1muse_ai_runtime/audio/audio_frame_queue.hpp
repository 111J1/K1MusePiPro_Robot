#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace k1muse_ai_runtime
{

template<typename T>
class AudioFrameQueue
{
public:
  explicit AudioFrameQueue(std::size_t capacity = 20);

  AudioFrameQueue(const AudioFrameQueue &) = delete;
  AudioFrameQueue & operator=(const AudioFrameQueue &) = delete;

  bool push(T frame);
  std::optional<T> try_pop();
  std::optional<T> pop_with_timeout(std::chrono::milliseconds timeout);
  // Block until data is available. Returns nullopt only if stopped.
  std::optional<T> wait_pop();
  // Wake up all waiting threads. After this, wait_pop returns nullopt.
  void stop();
  // Re-enable blocking waits after a lifecycle deactivate/activate round.
  void restart();
  std::size_t size() const;
  void clear();

private:
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::deque<T> queue_;
  bool stopped_{false};
};

template<typename T>
AudioFrameQueue<T>::AudioFrameQueue(std::size_t capacity)
: capacity_(capacity)
{
}

template<typename T>
bool AudioFrameQueue<T>::push(T frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.size() >= capacity_) {
    return false;
  }
  queue_.push_back(std::move(frame));
  not_empty_.notify_one();
  return true;
}

template<typename T>
std::optional<T> AudioFrameQueue<T>::try_pop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) {
    return std::nullopt;
  }
  T frame = std::move(queue_.front());
  queue_.pop_front();
  return frame;
}

template<typename T>
std::optional<T> AudioFrameQueue<T>::pop_with_timeout(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!not_empty_.wait_for(lock, timeout, [this]() { return !queue_.empty(); })) {
    return std::nullopt;
  }
  T frame = std::move(queue_.front());
  queue_.pop_front();
  return frame;
}

template<typename T>
std::optional<T> AudioFrameQueue<T>::wait_pop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  not_empty_.wait(lock, [this]() { return !queue_.empty() || stopped_; });
  if (queue_.empty()) {
    return std::nullopt;
  }
  T frame = std::move(queue_.front());
  queue_.pop_front();
  return frame;
}

template<typename T>
void AudioFrameQueue<T>::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = true;
  not_empty_.notify_all();
}

template<typename T>
void AudioFrameQueue<T>::restart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = false;
}

template<typename T>
std::size_t AudioFrameQueue<T>::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

template<typename T>
void AudioFrameQueue<T>::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
  not_empty_.notify_all();
}

}  // namespace k1muse_ai_runtime
