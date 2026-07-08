#include "k1muse_ai_runtime/voice_exclusive_guard.hpp"

namespace k1muse_ai_runtime
{

void VoiceExclusiveGuard::request_voice()
{
  std::lock_guard<std::mutex> lock(mutex_);
  voice_pending_ = true;
  // If no vision jobs are running, voice is immediately active.
  if (vision_in_flight_ == 0) {
    voice_active_ = true;
  }
}

void VoiceExclusiveGuard::release_voice()
{
  std::lock_guard<std::mutex> lock(mutex_);
  voice_pending_ = false;
  voice_active_ = false;
  cv_.notify_all();  // Wake vision side if it was waiting.
}

bool VoiceExclusiveGuard::wait_for_vision_drain(
    std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (vision_in_flight_ == 0) {
    voice_active_ = true;
    return true;
  }
  bool drained = cv_.wait_for(lock, timeout, [this]() {
    return vision_in_flight_ == 0;
  });
  if (drained) {
    voice_active_ = true;
  }
  return drained;
}

bool VoiceExclusiveGuard::can_submit_vision() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !voice_pending_;
}

void VoiceExclusiveGuard::on_vision_done()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (vision_in_flight_ > 0) {
    --vision_in_flight_;
  }
  if (vision_in_flight_ == 0) {
    if (voice_pending_) {
      voice_active_ = true;
    }
    cv_.notify_all();
  }
}

void VoiceExclusiveGuard::on_vision_started()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++vision_in_flight_;
  // A previously-submitted vision job is now running — voice must
  // wait for it to drain even if request_voice() already set
  // voice_active_ prematurely.
  if (voice_pending_ && vision_in_flight_ > 0) {
    voice_active_ = false;
  }
}

bool VoiceExclusiveGuard::voice_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return voice_active_;
}

int VoiceExclusiveGuard::vision_in_flight() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return vision_in_flight_;
}

}  // namespace k1muse_ai_runtime
