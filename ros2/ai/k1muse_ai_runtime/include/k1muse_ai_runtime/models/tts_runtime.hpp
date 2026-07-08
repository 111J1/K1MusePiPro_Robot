#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "k1muse_ai_runtime/backends/tts_backend.hpp"
#include "k1muse_ai_runtime/model_runtime.hpp"

namespace k1muse_ai_runtime
{

class TTSRuntime final : public ModelRuntime
{
public:
  using PcmResult = TtsBackend::Result;

  struct StatusCallback
  {
    // Called to publish TtsStatus
    std::function<void(
      const std::string & trace_id, const std::string & request_id,
      uint64_t epoch, const std::string & source,
      uint8_t state, const std::string & state_name,
      const std::string & reason)>
      publish_status;
    // Called to publish TtsPlayRequest with PCM
    std::function<void(
      const std::string & trace_id, const std::string & request_id,
      uint64_t epoch, const std::string & source,
      const PcmResult & pcm)>
      publish_play;
  };

  TTSRuntime(
    std::unique_ptr<TtsBackend> backend,
    const std::string & provider,
    StatusCallback callbacks);

  // ModelRuntime interface
  const std::string & name() const override;
  const std::string & provider() const override;
  void load(const CancellationToken & token, Deadline deadline) override;
  void warmup(const CancellationToken & token, Deadline deadline) override;
  void request_cancel() noexcept override;
  bool stop(std::chrono::milliseconds stop_timeout) noexcept override;
  void final_join() noexcept override;
  void unload() noexcept override;
  bool loaded() const noexcept override;

  /// Submit a TTS text request. Called from ROS callback.
  /// Returns true if accepted, false if rejected (lower priority than held request).
  /// If slot occupied: higher or equal priority replaces, lower priority rejected.
  bool submit_request(
    const std::string & trace_id, const std::string & request_id,
    uint64_t epoch, const std::string & source,
    uint8_t priority, const std::string & text, const std::string & voice);

  /// Process the pending request. Called by the long-running worker.
  /// Returns true if a request was processed, false if no pending or disabled.
  bool process_pending(
    const std::function<bool(uint64_t)> & epoch_is_current,
    const std::function<bool(JobModule)> & module_enabled,
    const std::function<bool()> & cancel_predicate,
    std::chrono::steady_clock::time_point deadline,
    uint64_t current_epoch, bool tts_enabled);

  /// Clear the pending slot.
  void clear_pending();

  /// Check if there's a pending request.
  bool has_pending() const;

private:
  struct PendingRequest
  {
    std::string trace_id;
    std::string request_id;
    uint64_t epoch{0};
    std::string source;
    uint8_t priority{0};
    std::string text;
    std::string voice;
  };

  std::unique_ptr<TtsBackend> backend_;
  std::string provider_;
  StatusCallback callbacks_;
  std::atomic<bool> stop_requested_{false};

  mutable std::mutex pending_mutex_;
  std::optional<PendingRequest> pending_;
};

}  // namespace k1muse_ai_runtime
