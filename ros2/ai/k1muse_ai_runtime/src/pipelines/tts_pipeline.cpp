#include "k1muse_ai_runtime/pipelines/tts_pipeline.hpp"

#include <chrono>
#include <thread>
#include <utility>

namespace k1muse_ai_runtime
{

TTSPipeline::TTSPipeline(
  std::unique_ptr<TtsBackend> tts_backend,
  const RuntimeConfig & config,
  StatusCallback callbacks)
: config_(config)
{
  tts_runtime_ = std::make_unique<TTSRuntime>(
    std::move(tts_backend),
    config_.tts_provider,
    TTSRuntime::StatusCallback{
      [cb = callbacks.publish_status](
        const std::string & trace_id, const std::string & request_id,
        uint64_t epoch, const std::string & source,
        uint8_t state, const std::string & state_name,
        const std::string & reason) {
        if (cb) {
          cb(trace_id, request_id, epoch, source, state, state_name, reason);
        }
      },
      [cb = callbacks.publish_play](
        const std::string & trace_id, const std::string & request_id,
        uint64_t epoch, const std::string & source,
        const PcmResult & pcm) {
        if (cb) {
          cb(trace_id, request_id, epoch, source, pcm);
        }
      }
    });
}

void TTSPipeline::load(const CancellationToken & token, Deadline deadline)
{
  tts_runtime_->load(token, deadline);
}

void TTSPipeline::warmup(const CancellationToken & token, Deadline deadline)
{
  tts_runtime_->warmup(token, deadline);
}

bool TTSPipeline::submit_request(
  const std::string & trace_id, const std::string & request_id,
  uint64_t epoch, const std::string & source,
  uint8_t priority, const std::string & text, const std::string & voice)
{
  return tts_runtime_->submit_request(
    trace_id, request_id, epoch, source, priority, text, voice);
}

InferenceJob TTSPipeline::build_tts_job(
  uint64_t epoch,
  const std::function<bool(uint64_t)> & epoch_is_current,
  const std::function<bool(JobModule)> & module_enabled,
  const std::function<bool()> & cancel_predicate,
  bool tts_enabled)
{
  auto * tts_ptr = tts_runtime_.get();
  const auto tts_module = (config_.tts_provider == "ep")
    ? JobModule::TTS_EP : JobModule::TTS_CPU;

  InferenceJob job;
  job.id = "tts:inference";
  job.module = tts_module;
  job.provider_class = canonical_provider(tts_module);
  job.priority = canonical_priority(tts_module);
  job.epoch = epoch;
  job.deadline = std::chrono::steady_clock::now() + std::chrono::hours(24);
  job.epoch_is_current = epoch_is_current;
  job.module_enabled = module_enabled;
  job.cancel_predicate = cancel_predicate;
  job.execute =
    [tts_ptr, epoch_is_current, cancel_predicate, tts_enabled](
      const CancellationToken & token) {
      while (!token.stop_requested()) {
        tts_ptr->process_pending(
          epoch_is_current,
          [](JobModule) { return true; },
          cancel_predicate,
          std::chrono::steady_clock::now() + std::chrono::milliseconds(100),
          0,  // epoch will be checked by commit_if_current
          tts_enabled);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

void TTSPipeline::clear_pending()
{
  if (tts_runtime_) {
    tts_runtime_->clear_pending();
  }
}

void TTSPipeline::request_cancel()
{
  if (tts_runtime_) {
    tts_runtime_->request_cancel();
  }
}

bool TTSPipeline::stop(std::chrono::milliseconds timeout)
{
  if (tts_runtime_) {
    return tts_runtime_->stop(timeout);
  }
  return true;
}

void TTSPipeline::final_join()
{
  if (tts_runtime_) {
    tts_runtime_->final_join();
  }
}

void TTSPipeline::unload()
{
  if (tts_runtime_) {
    tts_runtime_->unload();
  }
}

}  // namespace k1muse_ai_runtime
