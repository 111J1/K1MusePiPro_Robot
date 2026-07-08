#include "k1muse_ai_runtime/backends/real_tts_backend.hpp"

#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS

#include "tts/tts_service.h"

#include <iostream>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

struct RealTtsBackend::Impl
{
  std::string model_dir;
  std::string backend_type;
  std::string provider;
  bool is_loaded{false};

  SpacemiT::TtsConfig config;
  std::unique_ptr<SpacemiT::TtsEngine> engine;

  Impl(const std::string & dir, const std::string & type,
       const std::string & prov)
    : model_dir(dir), backend_type(type), provider(prov)
  {
  }
};

RealTtsBackend::RealTtsBackend(
  const std::string & model_dir,
  const std::string & backend_type,
  const std::string & provider)
  : impl_(std::make_unique<Impl>(model_dir, backend_type, provider))
{
}

RealTtsBackend::~RealTtsBackend() = default;

const std::string & RealTtsBackend::name() const
{
  static const std::string n{"real_tts"};
  return n;
}

void RealTtsBackend::load()
{
  if (impl_->is_loaded) {
    return;
  }

  impl_->config = SpacemiT::TtsConfig::Preset(impl_->backend_type);
  impl_->config.model_dir = impl_->model_dir;
  impl_->config.format = SpacemiT::AudioFormat::PCM;
  impl_->config.num_threads = 2;
  impl_->config.provider = impl_->provider;

  impl_->engine = std::make_unique<SpacemiT::TtsEngine>(impl_->config);

  if (!impl_->engine->IsInitialized()) {
    std::cerr << "[real_backend] real_tts load failed: "
              << "TtsEngine initialization failed (model_dir="
              << impl_->model_dir << ")" << std::endl;
    impl_->engine.reset();
    return;
  }

  impl_->is_loaded = true;
}

void RealTtsBackend::unload()
{
  impl_->engine.reset();
  impl_->is_loaded = false;
}

bool RealTtsBackend::loaded() const
{
  return impl_->is_loaded;
}

void RealTtsBackend::warmup()
{
  if (!impl_->is_loaded || !impl_->engine) {
    return;
  }
  // Perform a short synthesis to warm up the engine.
  SpacemiT::TtsConfig warmup_cfg = impl_->config;
  warmup_cfg.format = SpacemiT::AudioFormat::PCM;
  impl_->engine->Call("hello", warmup_cfg);
}

TtsBackend::Result RealTtsBackend::synthesize(
  const std::string & text, const std::string & voice)
{
  Result result;

  if (!impl_->is_loaded || !impl_->engine) {
    result.reason = "Backend not loaded";
    return result;
  }

  SpacemiT::TtsConfig synth_config = impl_->config;
  synth_config.format = SpacemiT::AudioFormat::PCM;
  if (!voice.empty()) {
    synth_config.voice = voice;
  }

  auto tts_result = impl_->engine->Call(text, synth_config);
  if (!tts_result) {
    result.reason = "TTS Call returned null";
    return result;
  }

  if (!tts_result->IsSuccess()) {
    result.reason = tts_result->GetMessage();
    return result;
  }

  result.success = true;
  result.pcm_s16le = tts_result->GetAudioInt16();
  result.sample_rate = static_cast<uint32_t>(tts_result->GetSampleRate());
  result.channels = 1;
  result.encoding = "pcm_s16le";

  return result;
}

void RealTtsBackend::reset()
{
  // TtsEngine does not expose a per-call reset; no action needed.
}

}  // namespace k1muse_ai_runtime

#else  // !K1MUSE_ENABLE_REAL_K1_BACKENDS

namespace k1muse_ai_runtime
{

struct RealTtsBackend::Impl {};

RealTtsBackend::RealTtsBackend(
  const std::string &, const std::string &, const std::string &) {}
RealTtsBackend::~RealTtsBackend() = default;

const std::string & RealTtsBackend::name() const
{
  static const std::string n{"real_tts(stub)"};
  return n;
}

void RealTtsBackend::load() {}
void RealTtsBackend::unload() {}
bool RealTtsBackend::loaded() const { return false; }
void RealTtsBackend::warmup() {}

TtsBackend::Result RealTtsBackend::synthesize(
  const std::string &, const std::string &)
{
  Result r;
  r.reason = "Real backends not enabled";
  return r;
}

void RealTtsBackend::reset() {}

}  // namespace k1muse_ai_runtime

#endif  // K1MUSE_ENABLE_REAL_K1_BACKENDS
