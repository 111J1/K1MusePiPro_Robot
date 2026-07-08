#pragma once

#include <condition_variable>
#include <mutex>

namespace k1muse_ai_runtime
{

/// Voice-exclusive guard for EP resource scheduling.
///
/// When voice (ASR/TTS) requests EP, new Vision jobs are blocked from
/// submission.  The currently-running Vision job completes naturally and
/// releases the lease — voice then obtains EP.  No preemption.
///
/// This is NOT a replacement for ResourceGuard.  It coordinates between
/// the voice and vision pipelines *before* jobs enter the scheduler.
///
/// Thread-safe.
class VoiceExclusiveGuard
{
public:
  VoiceExclusiveGuard() = default;

  // ── Voice side ──

  /// Request EP for voice.  New Vision submissions will be rejected
  /// until voice releases.
  void request_voice();

  /// Release the voice EP hold.  Vision may submit again.
  void release_voice();

  /// Wait until the currently-running Vision job has finished and the
  /// EP is available for voice.  Caller must have already called
  /// request_voice().
  ///
  /// Returns false if the timeout expires before the vision job
  /// finishes, true otherwise.
  bool wait_for_vision_drain(std::chrono::milliseconds timeout);

  // ── Vision side ──

  /// Check whether a new Vision job may be submitted.
  /// Returns false when voice has requested EP and the vision side
  /// should drain (stop submitting new jobs).
  bool can_submit_vision() const;

  /// Signal that the currently-running Vision job has finished
  /// (released the EP lease).  Wakes any voice thread waiting in
  /// wait_for_vision_drain().
  void on_vision_done();

  /// Signal that a new Vision job has started.  Increments the
  /// in-flight counter so wait_for_vision_drain() knows when the
  /// last job is done.
  void on_vision_started();

  // ── Query ──

  /// True when voice currently holds the exclusive right.
  bool voice_active() const;

  /// Number of Vision jobs currently in-flight.
  int vision_in_flight() const;

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;

  bool voice_pending_{false};   // voice has requested EP
  bool voice_active_{false};    // voice has drained vision and is active
  int vision_in_flight_{0};     // count of running Vision EP jobs
};

}  // namespace k1muse_ai_runtime
