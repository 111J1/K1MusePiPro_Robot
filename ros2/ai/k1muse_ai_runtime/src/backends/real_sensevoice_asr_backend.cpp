#include "k1muse_ai_runtime/backends/real_sensevoice_asr_backend.hpp"

#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS

// RISK: Using internal SDK header (backends/sensevoice/sensevoice_model.hpp) instead of
// the public asr_service.h API. This is deliberate: the public API pulls in libasr.a which
// depends on kaldifst/OpenFst -- libraries that are not installed on the K1 board. The
// SenseVoiceModel class is a self-contained alternative that only needs the ONNX model files.
// If the SDK changes the SenseVoiceModel interface in a future version, this backend will
// need updating. The proper fix is to get kaldifst packaged for the K1, then switch to the
// public asr_service.h API.
#include "backends/sensevoice/sensevoice_model.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

struct RealSenseVoiceAsrBackend::Impl
{
  std::string model_dir;
  std::string language;
  std::string provider;
  bool is_loaded{false};

  asr::sensevoice::SenseVoiceModel::Config config;
  std::unique_ptr<asr::sensevoice::SenseVoiceModel> model;

  Impl(const std::string & dir, const std::string & lang,
       const std::string & prov)
    : model_dir(dir), language(lang), provider(prov)
  {
  }
};

RealSenseVoiceAsrBackend::RealSenseVoiceAsrBackend(
  const std::string & model_dir,
  const std::string & language,
  const std::string & provider)
  : impl_(std::make_unique<Impl>(model_dir, language, provider))
{
}

RealSenseVoiceAsrBackend::~RealSenseVoiceAsrBackend() = default;

const std::string & RealSenseVoiceAsrBackend::name() const
{
  static const std::string n{"real_sensevoice_asr"};
  return n;
}

void RealSenseVoiceAsrBackend::load()
{
  if (impl_->is_loaded) {
    return;
  }

  impl_->config.model_path = impl_->model_dir + "/model_quant_optimized.onnx";
  impl_->config.cmvn_path = impl_->model_dir + "/am.mvn";
  impl_->config.vocab_path = impl_->model_dir + "/tokens.txt";
  impl_->config.language = impl_->language;
  impl_->config.provider = impl_->provider;

  impl_->model = std::make_unique<asr::sensevoice::SenseVoiceModel>(impl_->config);

  if (!impl_->model->initialize()) {
    std::cerr << "[real_backend] real_sensevoice_asr load failed: "
              << "SenseVoiceModel::initialize failed (model_dir="
              << impl_->model_dir << ")" << std::endl;
    impl_->model.reset();
    return;
  }

  impl_->is_loaded = true;
}

void RealSenseVoiceAsrBackend::unload()
{
  if (impl_->model) {
    impl_->model->shutdown();
    impl_->model.reset();
  }
  impl_->is_loaded = false;
}

bool RealSenseVoiceAsrBackend::loaded() const
{
  return impl_->is_loaded;
}

AsrBackend::Result RealSenseVoiceAsrBackend::transcribe(
  const std::vector<int16_t> & pcm, uint32_t /*sample_rate*/)
{
  Result result;

  if (!impl_->is_loaded || !impl_->model || pcm.empty()) {
    result.reason = "Backend not loaded or empty audio";
    return result;
  }

  // Convert int16 to float32 (normalize to [-1, 1])
  std::vector<float> float_pcm(pcm.size());
  for (size_t i = 0; i < pcm.size(); ++i) {
    float_pcm[i] = static_cast<float>(pcm[i]) / 32768.0f;
  }

  std::string text = impl_->model->recognize(float_pcm);

  result.text = text;
  result.language = impl_->language;
  result.confidence = text.empty() ? 0.0f : 1.0f;
  result.success = !text.empty();

  if (!result.success) {
    result.reason = "Empty recognition result";
  }

  return result;
}

void RealSenseVoiceAsrBackend::reset()
{
  // SenseVoiceModel does not have a per-stream reset;
  // no action needed for non-streaming use.
}

}  // namespace k1muse_ai_runtime

#else  // !K1MUSE_ENABLE_REAL_K1_BACKENDS

namespace k1muse_ai_runtime
{

struct RealSenseVoiceAsrBackend::Impl {};

RealSenseVoiceAsrBackend::RealSenseVoiceAsrBackend(
  const std::string &, const std::string &, const std::string &) {}

RealSenseVoiceAsrBackend::~RealSenseVoiceAsrBackend() = default;

const std::string & RealSenseVoiceAsrBackend::name() const
{
  static const std::string n{"real_sensevoice_asr(stub)"};
  return n;
}

void RealSenseVoiceAsrBackend::load() {}
void RealSenseVoiceAsrBackend::unload() {}
bool RealSenseVoiceAsrBackend::loaded() const { return false; }

AsrBackend::Result RealSenseVoiceAsrBackend::transcribe(
  const std::vector<int16_t> &, uint32_t)
{
  Result r;
  r.reason = "Real backends not enabled";
  return r;
}

void RealSenseVoiceAsrBackend::reset() {}

}  // namespace k1muse_ai_runtime

#endif  // K1MUSE_ENABLE_REAL_K1_BACKENDS
