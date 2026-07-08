#include "k1muse_ai_runtime/backends/real_vad_backend.hpp"

#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS

#include "vad_service.h"

#include <iostream>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

struct RealVadBackend::Impl
{
  std::string model_dir;
  float threshold;
  bool is_loaded{false};

  SpacemiT::VadConfig config;
  std::unique_ptr<SpacemiT::VadEngine> engine;

  Impl(const std::string & dir, float thresh)
    : model_dir(dir), threshold(thresh)
  {
  }
};

RealVadBackend::RealVadBackend(
  const std::string & model_dir,
  float threshold)
  : impl_(std::make_unique<Impl>(model_dir, threshold))
{
}

RealVadBackend::~RealVadBackend() = default;

const std::string & RealVadBackend::name() const
{
  static const std::string n{"real_vad"};
  return n;
}

void RealVadBackend::load()
{
  if (impl_->is_loaded) {
    return;
  }

  impl_->config = SpacemiT::VadConfig::Preset("silero");
  impl_->config.model_dir = impl_->model_dir;
  impl_->config.trigger_threshold = impl_->threshold;

  impl_->engine = std::make_unique<SpacemiT::VadEngine>(impl_->config);

  if (!impl_->engine->IsInitialized()) {
    std::cerr << "[real_backend] real_vad load failed: "
              << "VadEngine initialization failed (model_dir="
              << impl_->model_dir << ")" << std::endl;
    impl_->engine.reset();
    return;
  }

  impl_->is_loaded = true;
}

void RealVadBackend::unload()
{
  impl_->engine.reset();
  impl_->is_loaded = false;
}

bool RealVadBackend::loaded() const
{
  return impl_->is_loaded;
}

float RealVadBackend::process(const int16_t * pcm, size_t samples)
{
  if (!impl_->is_loaded || pcm == nullptr || samples == 0) {
    return 0.0f;
  }

  // Convert int16 to float32 (normalize to [-1, 1])
  std::vector<float> float_samples(samples);
  for (size_t i = 0; i < samples; ++i) {
    float_samples[i] = static_cast<float>(pcm[i]) / 32768.0f;
  }

  auto result = impl_->engine->Detect(float_samples.data(), samples);
  if (!result) {
    return 0.0f;
  }

  return result->GetProbability();
}

void RealVadBackend::reset()
{
  if (impl_->is_loaded && impl_->engine) {
    impl_->engine->Reset();
  }
}

}  // namespace k1muse_ai_runtime

#else  // !K1MUSE_ENABLE_REAL_K1_BACKENDS

namespace k1muse_ai_runtime
{

struct RealVadBackend::Impl {};

RealVadBackend::RealVadBackend(const std::string &, float) {}
RealVadBackend::~RealVadBackend() = default;

const std::string & RealVadBackend::name() const
{
  static const std::string n{"real_vad(stub)"};
  return n;
}

void RealVadBackend::load() {}
void RealVadBackend::unload() {}
bool RealVadBackend::loaded() const { return false; }
float RealVadBackend::process(const int16_t *, size_t) { return 0.0f; }
void RealVadBackend::reset() {}

}  // namespace k1muse_ai_runtime

#endif  // K1MUSE_ENABLE_REAL_K1_BACKENDS
