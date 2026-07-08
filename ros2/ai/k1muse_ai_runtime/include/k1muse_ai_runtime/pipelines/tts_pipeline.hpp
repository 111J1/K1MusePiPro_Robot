#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "k1muse_ai_runtime/backends/tts_backend.hpp"
#include "k1muse_ai_runtime/inference_job.hpp"
#include "k1muse_ai_runtime/model_runtime.hpp"
#include "k1muse_ai_runtime/models/tts_runtime.hpp"
#include "k1muse_ai_runtime/runtime_config.hpp"

namespace k1muse_ai_runtime
{

using Deadline = std::chrono::steady_clock::time_point;

/// TTS pipeline encapsulating TTSRuntime and playback management.
/// Extracted from AiRuntimeNode for modularity.
class TTSPipeline
{
public:
  using PcmResult = TtsBackend::Result;

  struct StatusCallback
  {
    std::function<void(
      const std::string & trace_id, const std::string & request_id,
      uint64_t epoch, const std::string & source,
      uint8_t state, const std::string & state_name,
      const std::string & reason)>
      publish_status;

    std::function<void(
      const std::string & trace_id, const std::string & request_id,
      uint64_t epoch, const std::string & source,
      const PcmResult & pcm)>
      publish_play;
  };

  TTSPipeline(
    std::unique_ptr<TtsBackend> tts_backend,
    const RuntimeConfig & config,
    StatusCallback callbacks);

  ~TTSPipeline() = default;

  TTSPipeline(const TTSPipeline &) = delete;
  TTSPipeline & operator=(const TTSPipeline &) = delete;

  /// Load the TTS backend.
  void load(const CancellationToken & token, Deadline deadline);

  /// Warmup the TTS backend.
  void warmup(const CancellationToken & token, Deadline deadline);

  /// Submit a TTS text request.
  bool submit_request(
    const std::string & trace_id, const std::string & request_id,
    uint64_t epoch, const std::string & source,
    uint8_t priority, const std::string & text, const std::string & voice);

  /// Build the TTS InferenceJob for the given epoch.
  InferenceJob build_tts_job(
    uint64_t epoch,
    const std::function<bool(uint64_t)> & epoch_is_current,
    const std::function<bool(JobModule)> & module_enabled,
    const std::function<bool()> & cancel_predicate,
    bool tts_enabled);

  /// Get the TTS runtime (for warmup).
  TTSRuntime * tts_runtime() const { return tts_runtime_.get(); }

  /// Clear pending requests.
  void clear_pending();

  /// Request cancel on the runtime.
  void request_cancel();

  /// Stop the runtime.
  bool stop(std::chrono::milliseconds timeout);

  /// Final join on the runtime.
  void final_join();

  /// Unload the backend.
  void unload();

private:
  std::unique_ptr<TTSRuntime> tts_runtime_;
  RuntimeConfig config_;
};

}  // namespace k1muse_ai_runtime
