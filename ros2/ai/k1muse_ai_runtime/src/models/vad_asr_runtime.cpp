#include "k1muse_ai_runtime/models/vad_asr_runtime.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

#include <mutex>  // IWYU pragma: keep (already in header, but explicit for clarity)

namespace k1muse_ai_runtime
{

VadAsrRuntime::VadAsrRuntime(
  std::unique_ptr<VadBackend> vad_backend,
  std::unique_ptr<AsrBackend> asr_backend,
  VadSegmenter::Config segmenter_config,
  const std::string & asr_provider,
  ListenEventCallback event_callback,
  ListenResultCallback result_callback,
  SegmentReadyCallback segment_callback)
: vad_backend_(std::move(vad_backend)),
  asr_backend_(std::move(asr_backend)),
  segmenter_(segmenter_config),
  asr_provider_(asr_provider),
  event_callback_(std::move(event_callback)),
  result_callback_(std::move(result_callback)),
  segment_callback_(std::move(segment_callback))
{
}

const std::string & VadAsrRuntime::name() const
{
  static const std::string n{"vad_asr"};
  return n;
}

const std::string & VadAsrRuntime::provider() const
{
  return asr_provider_;
}

void VadAsrRuntime::load(const CancellationToken & token, Deadline deadline)
{
  if (token.stop_requested() || std::chrono::steady_clock::now() > deadline) {
    throw std::runtime_error("vad_asr load cancelled or expired");
  }
  stop_requested_.store(false);
  vad_backend_->load();
  asr_backend_->load();
}

void VadAsrRuntime::warmup(const CancellationToken & token, Deadline deadline)
{
  if (!vad_backend_->loaded() || !asr_backend_->loaded()) {
    throw std::runtime_error("vad_asr backends are not loaded");
  }
  const auto warmup_end =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
  while (std::chrono::steady_clock::now() < warmup_end) {
    if (token.stop_requested() || stop_requested_.load() ||
      std::chrono::steady_clock::now() > deadline)
    {
      throw std::runtime_error("vad_asr warmup cancelled");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void VadAsrRuntime::request_cancel() noexcept
{
  stop_requested_.store(true);
}

bool VadAsrRuntime::stop(std::chrono::milliseconds /*stop_timeout*/) noexcept
{
  stop_requested_.store(true);
  std::lock_guard<std::mutex> lock(turn_mutex_);
  segmenter_.reset();
  speech_active_ = false;
  current_utterance_id_.clear();
  speech_start_epoch_ = 0;
  return true;
}

void VadAsrRuntime::final_join() noexcept
{
  // No threads owned by this runtime.
}

void VadAsrRuntime::unload() noexcept
{
  if (vad_backend_) {
    vad_backend_->unload();
  }
  if (asr_backend_) {
    asr_backend_->unload();
  }
}

bool VadAsrRuntime::loaded() const noexcept
{
  return vad_backend_ && vad_backend_->loaded() &&
         asr_backend_ && asr_backend_->loaded();
}

void VadAsrRuntime::reset_turn()
{
  std::lock_guard<std::mutex> lock(turn_mutex_);
  segmenter_.reset();
  speech_active_ = false;
  current_utterance_id_.clear();
  speech_start_epoch_ = 0;
  seq_initialized_ = false;
}

VadSegmenter::State VadAsrRuntime::vad_state() const
{
  return segmenter_.state();
}

void VadAsrRuntime::process_audio(
  const int16_t * pcm, size_t samples,
  const std::string & trace_id, uint64_t epoch,
  const ControlSnapshot & control, uint64_t seq)
{
  if (stop_requested_.load() || !control.vad_asr_enabled) {
    // Keep last_seq_ current even when VAD is disabled so the seq-gap
    // detection on re-enable has the right baseline.  Without this,
    // reset_current_turn() mid-gap can leave seq_initialized_=false,
    // bypassing the gap recovery and starting VAD with zero pre-roll.
    last_seq_ = seq;
    seq_initialized_ = true;
    return;
  }

  std::lock_guard<std::mutex> lock(turn_mutex_);

  // Sequence gap detection: reset VAD on out-of-order or dropped frames.
  if (seq_initialized_ && seq != last_seq_ + 1) {
    segmenter_.reset();
    if (vad_backend_) { vad_backend_->reset(); }
    speech_active_ = false;
    current_utterance_id_.clear();
  }
  last_seq_ = seq;
  seq_initialized_ = true;

  if (!vad_backend_ || !vad_backend_->loaded() ||
    !asr_backend_ || !asr_backend_->loaded())
  {
    return;
  }

  // Feed audio through VAD backend.
  const float vad_probability = vad_backend_->process(pcm, samples);

  // Feed PCM and probability to the segmenter.
  const VadSegmenter::SegmentResult seg_result =
    segmenter_.process(pcm, samples, vad_probability);

  const VadSegmenter::State current_state = segmenter_.state();

  // Detect speech start: transition from Armed to PreSpeech.
  if (!speech_active_ &&
    (current_state == VadSegmenter::State::PreSpeech ||
     current_state == VadSegmenter::State::InSpeech ||
     current_state == VadSegmenter::State::EndingSilence))
  {
    speech_active_ = true;
    speech_start_epoch_ = epoch;
    current_utterance_id_ = "utt_" + std::to_string(
      utterance_counter_.fetch_add(1) + 1);
    if (event_callback_) {
      event_callback_(
        trace_id, current_utterance_id_, speech_start_epoch_,
        ListenEvent::SPEECH_START, "");
    }
  }

  // Process segment-ready event.
  if (seg_result.ready && speech_active_) {
    const std::string utterance_id = current_utterance_id_;
    const uint64_t segment_epoch = speech_start_epoch_;

    // Fire SPEECH_END.
    if (event_callback_) {
      event_callback_(
        trace_id, utterance_id, segment_epoch,
        ListenEvent::SPEECH_END, "");
    }

    // Fire ASR_STARTED before transcription.
    if (event_callback_) {
      event_callback_(
        trace_id, utterance_id, segment_epoch,
        ListenEvent::ASR_STARTED, "");
    }

    // Run ASR on the complete segment.
    const AsrBackend::Result asr_result =
      asr_backend_->transcribe(seg_result.pcm, seg_result.sample_rate);

    // Fire ASR_DONE after successful transcription.
    if (event_callback_ && asr_result.success) {
      event_callback_(
        trace_id, utterance_id, segment_epoch,
        ListenEvent::ASR_DONE, "");
    }

    // Epoch gate: drop result if epoch changed since speech started.
    if (speech_start_epoch_ == epoch) {
      if (result_callback_) {
        result_callback_(
          trace_id, utterance_id, segment_epoch,
          asr_result.success, asr_result.text,
          asr_result.confidence, asr_result.language,
          asr_result.reason);
      }
    }

    // Reset for next utterance.
    speech_active_ = false;
    current_utterance_id_.clear();
    speech_start_epoch_ = 0;
  }
}

void VadAsrRuntime::process_vad_only(
  const int16_t * pcm, size_t samples,
  const std::string & trace_id, uint64_t epoch,
  const ControlSnapshot & control, uint64_t seq)
{
  if (stop_requested_.load()) {
    return;
  }

  // When VAD is disabled: still feed pre-roll buffer so the segmenter
  // has 300ms of fresh audio context when VAD re-enables (next LISTENING).
  if (!control.vad_asr_enabled) {
    std::lock_guard<std::mutex> lock(turn_mutex_);
    if (speech_active_) {
      segmenter_.do_reset();
      if (vad_backend_) { vad_backend_->reset(); }
      speech_active_ = false;
      current_utterance_id_.clear();
    }
    segmenter_.feed_pre_roll(pcm, samples);
    last_seq_ = seq;
    seq_initialized_ = true;
    vad_was_disabled_ = true;
    return;
  }

  std::lock_guard<std::mutex> lock(turn_mutex_);

  // On VAD re-enable: clear active segment state (mid-utterance residue)
  // but keep the pre-roll ring - it has 300ms of fresh audio from above.
  if (vad_was_disabled_) {
    segmenter_.do_reset();
    if (vad_backend_) { vad_backend_->reset(); }
    speech_active_ = false;
    current_utterance_id_.clear();
    vad_was_disabled_ = false;
  }

  // Sequence gap detection: reset VAD on out-of-order or dropped frames.
  if (seq_initialized_ && seq != last_seq_ + 1) {
    segmenter_.reset();
    if (vad_backend_) { vad_backend_->reset(); }
    speech_active_ = false;
    current_utterance_id_.clear();
  }
  last_seq_ = seq;
  seq_initialized_ = true;

  if (!vad_backend_ || !vad_backend_->loaded()) {
    static int logged = 0;
    if (logged < 5) {
      fprintf(stderr, "[vad_asr] VAD backend not loaded (vad_backend_=%p, loaded=%d). "
              "No speech detection will occur.\n",
              static_cast<const void*>(vad_backend_.get()),
              vad_backend_ ? static_cast<int>(vad_backend_->loaded()) : -1);
      ++logged;
    }
    return;
  }

  // Feed audio through VAD backend.
  const float vad_probability = vad_backend_->process(pcm, samples);

  // Feed PCM and probability to the segmenter.
  const VadSegmenter::SegmentResult seg_result =
    segmenter_.process(pcm, samples, vad_probability);

  const VadSegmenter::State current_state = segmenter_.state();

  // Detect speech start: transition from Armed to PreSpeech.
  if (!speech_active_ &&
    (current_state == VadSegmenter::State::PreSpeech ||
     current_state == VadSegmenter::State::InSpeech ||
     current_state == VadSegmenter::State::EndingSilence))
  {
    speech_active_ = true;
    speech_start_epoch_ = epoch;
    current_utterance_id_ = "utt_" + std::to_string(
      utterance_counter_.fetch_add(1) + 1);
    if (event_callback_) {
      event_callback_(
        trace_id, current_utterance_id_, speech_start_epoch_,
        ListenEvent::SPEECH_START, "");
    }
  }

  // Process segment-ready event: fire SPEECH_END then hand off to ASR via callback.
  if (seg_result.ready && speech_active_) {
    // Fire SPEECH_END.
    if (event_callback_) {
      event_callback_(
        trace_id, current_utterance_id_, speech_start_epoch_,
        ListenEvent::SPEECH_END, "");
    }

    // Build SegmentData and hand off to ASR via callback.
    SegmentData seg;
    seg.pcm = seg_result.pcm;
    seg.sample_rate = seg_result.sample_rate;
    seg.trace_id = trace_id;
    seg.utterance_id = current_utterance_id_;
    seg.epoch = speech_start_epoch_;

    speech_active_ = false;

    if (segment_callback_) {
      segment_callback_(std::move(seg));
    }
  }
}

void VadAsrRuntime::run_asr(const SegmentData & segment)
{
  if (stop_requested_.load()) {
    return;
  }

  // Fire ASR_STARTED event.
  if (event_callback_) {
    event_callback_(
      segment.trace_id, segment.utterance_id, segment.epoch,
      ListenEvent::ASR_STARTED, "");
  }

  // Run ASR on the complete segment.
  const AsrBackend::Result asr_result =
    asr_backend_->transcribe(segment.pcm, segment.sample_rate);

  // Fire ASR_DONE event.
  if (event_callback_) {
    event_callback_(
      segment.trace_id, segment.utterance_id, segment.epoch,
      ListenEvent::ASR_DONE, asr_result.success ? "" : asr_result.reason);
  }

  // Publish result directly. Epoch gating is handled by the InferenceJob's
  // commit_if_current via try_commit() at the job level.
  if (result_callback_) {
    result_callback_(
      segment.trace_id, segment.utterance_id, segment.epoch,
      asr_result.success, asr_result.text,
      asr_result.confidence, asr_result.language,
      asr_result.reason);
  }
}

}  // namespace k1muse_ai_runtime
