#include "k1muse_ai_runtime/pipelines/voice_pipeline.hpp"

#include <chrono>
#include <utility>

namespace k1muse_ai_runtime
{

VoicePipeline::VoicePipeline(
  std::unique_ptr<WakewordBackend> wakeword_backend,
  std::unique_ptr<VadBackend> vad_backend,
  std::unique_ptr<AsrBackend> asr_backend,
  const RuntimeConfig & config,
  WakewordCallback wakeword_callback,
  ListenEventCallback listen_event_callback,
  ListenResultCallback listen_result_callback,
  SegmentReadyCallback segment_callback)
: config_(config)
{
  // Create wakeword runtime
  wakeword_runtime_ = std::make_unique<WakewordRuntime>(
    std::move(wakeword_backend),
    [this, cb = std::move(wakeword_callback)](
      const std::string & trace_id, uint64_t epoch,
      float confidence, const std::string & keyword) {
      if (confidence >= config_.wakeword_threshold) {
        cb(trace_id, epoch, confidence, keyword);
      }
    });

  // Create VAD+ASR runtime
  VadSegmenter::Config seg_config;
  seg_config.sample_rate = 16000;
  seg_config.audio_chunk_ms = 20;
  seg_config.vad_threshold = config_.vad_threshold;
  seg_config.min_speech_ms = static_cast<uint32_t>(config_.min_speech_ms);
  seg_config.end_silence_ms = static_cast<uint32_t>(config_.end_silence_ms);
  seg_config.pre_roll_ms = static_cast<uint32_t>(config_.pre_roll_ms);
  seg_config.post_pad_ms = static_cast<uint32_t>(config_.post_pad_ms);
  seg_config.max_utterance_ms = static_cast<uint32_t>(config_.max_utterance_ms);

  vad_asr_runtime_ = std::make_unique<VadAsrRuntime>(
    std::move(vad_backend), std::move(asr_backend), seg_config,
    config_.asr_provider,
    std::move(listen_event_callback),
    std::move(listen_result_callback),
    std::move(segment_callback));

  // Create audio frame validator
  audio_validator_ = std::make_unique<AudioFrameValidator>();
}

bool VoicePipeline::process_audio_frame(
  const AudioFrameData & frame, const ControlSnapshot & control)
{
  const auto now = std::chrono::steady_clock::now();

  const auto result = audio_validator_->validate(
    16000, 1, "s16le", 20, frame.pcm, frame.seq, now, now);

  if (!result.valid || result.seq_gap) {
    return false;
  }

  AudioFrameData ww_frame = frame;
  if (!wakeword_queue_.push(std::move(ww_frame))) {
    return false;
  }

  AudioFrameData vad_frame = frame;
  if (!vad_asr_queue_.push(std::move(vad_frame))) {
    return false;
  }

  return true;
}

void VoicePipeline::load(const CancellationToken & token, Deadline deadline)
{
  wakeword_runtime_->load(token, deadline);
  vad_asr_runtime_->load(token, deadline);
}

void VoicePipeline::warmup(const CancellationToken & token, Deadline deadline)
{
  wakeword_runtime_->warmup(token, deadline);
  // Warmup VAD
  vad_asr_runtime_->warmup(token, deadline);
}

InferenceJob VoicePipeline::build_wakeword_job(
  uint64_t epoch,
  const std::function<bool(uint64_t)> & epoch_is_current,
  const std::function<bool(JobModule)> & module_enabled,
  const std::function<bool()> & cancel_predicate)
{
  auto * wakeword_ptr = wakeword_runtime_.get();
  InferenceJob job;
  job.id = "audio:wakeword";
  job.module = JobModule::WAKEWORD_CPU;
  job.provider_class = canonical_provider(JobModule::WAKEWORD_CPU);
  job.priority = canonical_priority(JobModule::WAKEWORD_CPU);
  job.epoch = epoch;
  job.deadline = std::chrono::steady_clock::now() + std::chrono::hours(24);
  job.epoch_is_current = epoch_is_current;
  job.module_enabled = module_enabled;
  job.cancel_predicate = cancel_predicate;
  job.execute =
    [this, wakeword_ptr, epoch_is_current](const CancellationToken & token) {
      while (!token.stop_requested()) {
        auto frame = wakeword_queue_.pop_with_timeout(
          std::chrono::milliseconds(50));
        if (frame) {
          ControlSnapshot control;
          control.epoch = frame->epoch;
          wakeword_ptr->process_audio(
            frame->pcm.data(), frame->samples,
            frame->trace_id, frame->epoch, control);
        }
      }
    };
  job.commit_if_current =
    [epoch, cancel_predicate](const CancellationToken & token) {
      (void)epoch;
      (void)cancel_predicate;
      return true;
    };
  return job;
}

InferenceJob VoicePipeline::build_vad_job(
  uint64_t epoch,
  const std::function<bool(uint64_t)> & epoch_is_current,
  const std::function<bool(JobModule)> & module_enabled,
  const std::function<bool()> & cancel_predicate)
{
  auto * vad_asr_ptr = vad_asr_runtime_.get();
  InferenceJob job;
  job.id = "audio:vad_asr";
  job.module = JobModule::VAD_CPU;
  job.provider_class = canonical_provider(JobModule::VAD_CPU);
  job.priority = canonical_priority(JobModule::VAD_CPU);
  job.epoch = epoch;
  job.deadline = std::chrono::steady_clock::now() + std::chrono::hours(24);
  job.epoch_is_current = epoch_is_current;
  job.module_enabled = module_enabled;
  job.cancel_predicate = cancel_predicate;
  job.execute =
    [this, vad_asr_ptr, epoch_is_current](const CancellationToken & token) {
      while (!token.stop_requested()) {
        auto frame = vad_asr_queue_.pop_with_timeout(
          std::chrono::milliseconds(50));
        if (frame) {
          ControlSnapshot control;
          control.epoch = frame->epoch;
          vad_asr_ptr->process_vad_only(
            frame->pcm.data(), frame->samples,
            frame->trace_id, frame->epoch, control, frame->seq);
        }
      }
    };
  job.commit_if_current =
    [epoch, cancel_predicate](const CancellationToken & token) {
      (void)epoch;
      (void)cancel_predicate;
      return true;
    };
  return job;
}

void VoicePipeline::reset()
{
  wakeword_queue_.clear();
  vad_asr_queue_.clear();
  if (audio_validator_) {
    audio_validator_->reset();
  }
  if (vad_asr_runtime_) {
    vad_asr_runtime_->reset_turn();
  }
}

void VoicePipeline::request_cancel()
{
  if (wakeword_runtime_) {
    wakeword_runtime_->request_cancel();
  }
  if (vad_asr_runtime_) {
    vad_asr_runtime_->request_cancel();
  }
}

bool VoicePipeline::stop(std::chrono::milliseconds timeout)
{
  bool stopped = true;
  if (wakeword_runtime_) {
    stopped = wakeword_runtime_->stop(timeout) && stopped;
  }
  if (vad_asr_runtime_) {
    stopped = vad_asr_runtime_->stop(timeout) && stopped;
  }
  return stopped;
}

void VoicePipeline::final_join()
{
  if (wakeword_runtime_) {
    wakeword_runtime_->final_join();
  }
  if (vad_asr_runtime_) {
    vad_asr_runtime_->final_join();
  }
}

void VoicePipeline::unload()
{
  if (wakeword_runtime_) {
    wakeword_runtime_->unload();
  }
  if (vad_asr_runtime_) {
    vad_asr_runtime_->unload();
  }
}

}  // namespace k1muse_ai_runtime
