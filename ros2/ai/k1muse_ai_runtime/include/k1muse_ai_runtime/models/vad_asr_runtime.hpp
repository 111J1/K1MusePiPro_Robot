#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "k1muse_ai_runtime/audio/vad_segmenter.hpp"
#include "k1muse_ai_runtime/backends/asr_backend.hpp"
#include "k1muse_ai_runtime/backends/vad_backend.hpp"
#include "k1muse_ai_runtime/control_snapshot.hpp"
#include "k1muse_ai_runtime/model_runtime.hpp"

namespace k1muse_ai_runtime
{

class VadAsrRuntime final : public ModelRuntime
{
public:
  enum class ListenEvent : uint8_t
  {
    SPEECH_START = 0,
    SPEECH_END = 1,
    ASR_STARTED = 2,
    ASR_DONE = 3,
    FAILED = 4,
    CANCELLED = 5,
  };

  using ListenEventCallback = std::function<void(
    const std::string & trace_id, const std::string & utterance_id,
    uint64_t epoch, ListenEvent event, const std::string & reason)>;

  using ListenResultCallback = std::function<void(
    const std::string & trace_id, const std::string & utterance_id,
    uint64_t epoch, bool success, const std::string & text,
    float confidence, const std::string & language,
    const std::string & reason)>;

  struct SegmentData
  {
    std::vector<int16_t> pcm;
    uint32_t sample_rate;
    std::string trace_id;
    std::string utterance_id;
    uint64_t epoch;
  };

  using SegmentReadyCallback = std::function<void(SegmentData segment)>;

  VadAsrRuntime(
    std::unique_ptr<VadBackend> vad_backend,
    std::unique_ptr<AsrBackend> asr_backend,
    VadSegmenter::Config segmenter_config,
    const std::string & asr_provider,
    ListenEventCallback event_callback,
    ListenResultCallback result_callback,
    SegmentReadyCallback segment_callback = {});

  const std::string & name() const override;
  const std::string & provider() const override;

  void load(const CancellationToken & token, Deadline deadline) override;
  void warmup(const CancellationToken & token, Deadline deadline) override;

  void request_cancel() noexcept override;
  bool stop(std::chrono::milliseconds stop_timeout) noexcept override;
  void final_join() noexcept override;
  void unload() noexcept override;
  bool loaded() const noexcept override;

  /// Process one audio chunk through VAD + ASR pipeline.
  /// Sequence gap detection resets the VAD segmenter.
  void process_audio(
    const int16_t * pcm, size_t samples,
    const std::string & trace_id, uint64_t epoch,
    const ControlSnapshot & control, uint64_t seq);

  /// VAD-only phase: runs VAD and fires SegmentReady callback when a segment
  /// is complete. Called from the VAD_CPU worker thread.
  void process_vad_only(
    const int16_t * pcm, size_t samples,
    const std::string & trace_id, uint64_t epoch,
    const ControlSnapshot & control, uint64_t seq);

  /// ASR-only phase: transcribes a completed segment and publishes the result
  /// via result_callback_. Called from an ASR InferenceJob execute.
  void run_asr(const SegmentData & segment);

  /// Reset turn-level processing state without stopping the runtime.
  /// Safe to call from the audio callback thread on seq-gap or queue-full.
  void reset_turn();

  /// Current VAD segmenter state (for testing).
  VadSegmenter::State vad_state() const;

private:
  std::unique_ptr<VadBackend> vad_backend_;
  std::unique_ptr<AsrBackend> asr_backend_;
  VadSegmenter segmenter_;
  std::string asr_provider_;
  ListenEventCallback event_callback_;
  ListenResultCallback result_callback_;
  SegmentReadyCallback segment_callback_;

  // Mutex protecting segmenter_ and turn-level state accessed from multiple threads.
  mutable std::mutex turn_mutex_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<uint64_t> utterance_counter_{0};
  uint64_t last_seq_{0};
  bool seq_initialized_{false};
  std::string current_utterance_id_;
  bool speech_active_{false};
  uint64_t speech_start_epoch_{0};
  bool vad_was_disabled_{false};  // reset segmenter on VAD re-enable
};

}  // namespace k1muse_ai_runtime
