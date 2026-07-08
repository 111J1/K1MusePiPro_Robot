#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "k1muse_ai_runtime/audio/audio_frame_queue.hpp"
#include "k1muse_ai_runtime/audio/audio_frame_validator.hpp"
#include "k1muse_ai_runtime/audio/vad_segmenter.hpp"
#include "k1muse_ai_runtime/backends/asr_backend.hpp"
#include "k1muse_ai_runtime/backends/vad_backend.hpp"
#include "k1muse_ai_runtime/backends/wakeword_backend.hpp"
#include "k1muse_ai_runtime/control_snapshot.hpp"
#include "k1muse_ai_runtime/inference_job.hpp"
#include "k1muse_ai_runtime/models/vad_asr_runtime.hpp"
#include "k1muse_ai_runtime/models/wakeword_runtime.hpp"
#include "k1muse_ai_runtime/model_runtime.hpp"
#include "k1muse_ai_runtime/runtime_config.hpp"

namespace k1muse_ai_runtime
{

using Deadline = std::chrono::steady_clock::time_point;

/// Voice pipeline encapsulating Wakeword + VAD + ASR processing.
/// Extracted from AiRuntimeNode for modularity.
class VoicePipeline
{
public:
  struct AudioFrameData
  {
    std::vector<int16_t> pcm;
    size_t samples{0};
    std::string trace_id;
    uint64_t epoch{0};
    uint64_t seq{0};
  };

  using WakewordCallback = std::function<void(
    const std::string & trace_id, uint64_t epoch,
    float confidence, const std::string & keyword)>;

  using ListenEventCallback = std::function<void(
    const std::string & trace_id, const std::string & utterance_id,
    uint64_t epoch, VadAsrRuntime::ListenEvent event,
    const std::string & reason)>;

  using ListenResultCallback = std::function<void(
    const std::string & trace_id, const std::string & utterance_id,
    uint64_t epoch, bool success, const std::string & text,
    float confidence, const std::string & language,
    const std::string & reason)>;

  using SegmentReadyCallback = std::function<void(VadAsrRuntime::SegmentData segment)>;

  VoicePipeline(
    std::unique_ptr<WakewordBackend> wakeword_backend,
    std::unique_ptr<VadBackend> vad_backend,
    std::unique_ptr<AsrBackend> asr_backend,
    const RuntimeConfig & config,
    WakewordCallback wakeword_callback,
    ListenEventCallback listen_event_callback,
    ListenResultCallback listen_result_callback,
    SegmentReadyCallback segment_callback);

  ~VoicePipeline() = default;

  VoicePipeline(const VoicePipeline &) = delete;
  VoicePipeline & operator=(const VoicePipeline &) = delete;

  /// Process an audio frame: validate, then dispatch to wakeword and vad_asr queues.
  /// Returns true if the frame was processed successfully, false on validation failure.
  bool process_audio_frame(const AudioFrameData & frame, const ControlSnapshot & control);

  /// Load all voice backends.
  void load(const CancellationToken & token, Deadline deadline);

  /// Warmup all voice backends.
  void warmup(const CancellationToken & token, Deadline deadline);

  /// Build the wakeword InferenceJob for the given epoch.
  InferenceJob build_wakeword_job(
    uint64_t epoch,
    const std::function<bool(uint64_t)> & epoch_is_current,
    const std::function<bool(JobModule)> & module_enabled,
    const std::function<bool()> & cancel_predicate);

  /// Build the VAD InferenceJob for the given epoch.
  InferenceJob build_vad_job(
    uint64_t epoch,
    const std::function<bool(uint64_t)> & epoch_is_current,
    const std::function<bool(JobModule)> & module_enabled,
    const std::function<bool()> & cancel_predicate);

  /// Get the wakeword runtime (for warmup).
  WakewordRuntime * wakeword_runtime() const { return wakeword_runtime_.get(); }

  /// Get the VAD+ASR runtime (for warmup).
  VadAsrRuntime * vad_asr_runtime() const { return vad_asr_runtime_.get(); }

  /// Clear all queues and reset state.
  void reset();

  /// Request cancel on all runtimes.
  void request_cancel();

  /// Stop all runtimes.
  bool stop(std::chrono::milliseconds timeout);

  /// Final join on all runtimes.
  void final_join();

  /// Unload all backends.
  void unload();

private:
  std::unique_ptr<WakewordRuntime> wakeword_runtime_;
  std::unique_ptr<VadAsrRuntime> vad_asr_runtime_;
  std::unique_ptr<AudioFrameValidator> audio_validator_;

  AudioFrameQueue<AudioFrameData> wakeword_queue_{20};
  AudioFrameQueue<AudioFrameData> vad_asr_queue_{20};

  RuntimeConfig config_;
};

}  // namespace k1muse_ai_runtime
