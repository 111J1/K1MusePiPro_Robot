#include "k1muse_ai_runtime/backends/real_sherpa_wakeword_backend.hpp"

#ifdef K1MUSE_ENABLE_REAL_K1_BACKENDS

#include "sherpa-onnx/c-api/cxx-api.h"

#include <iostream>
#include <string>
#include <vector>

namespace k1muse_ai_runtime
{

struct RealSherpaWakewordBackend::Impl
{
  std::string model_dir;
  float threshold;
  std::string keywords_file;
  bool is_loaded{false};

  sherpa_onnx::cxx::KeywordSpotterConfig config;
  std::unique_ptr<sherpa_onnx::cxx::KeywordSpotter> spotter;
  std::unique_ptr<sherpa_onnx::cxx::OnlineStream> stream;

  Impl(const std::string & dir, float thresh, const std::string & kw_file)
    : model_dir(dir), threshold(thresh), keywords_file(kw_file)
  {
  }
};

RealSherpaWakewordBackend::RealSherpaWakewordBackend(
  const std::string & model_dir,
  float threshold,
  const std::string & keywords_file)
  : impl_(std::make_unique<Impl>(model_dir, threshold, keywords_file))
{
}

RealSherpaWakewordBackend::~RealSherpaWakewordBackend() = default;

const std::string & RealSherpaWakewordBackend::name() const
{
  static const std::string n{"real_sherpa_wakeword"};
  return n;
}

void RealSherpaWakewordBackend::load()
{
  if (impl_->is_loaded) {
    return;
  }

  impl_->config.model_config.model_type = "zipformer2";
  impl_->config.model_config.tokens = impl_->model_dir + "/tokens.txt";
  impl_->config.model_config.transducer.encoder =
    impl_->model_dir + "/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx";
  impl_->config.model_config.transducer.decoder =
    impl_->model_dir + "/decoder-epoch-13-avg-2-chunk-16-left-64.onnx";
  impl_->config.model_config.transducer.joiner =
    impl_->model_dir + "/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx";
  impl_->config.model_config.num_threads = 1;
  impl_->config.model_config.provider = "cpu";
  impl_->config.model_config.debug = false;
  impl_->config.feat_config.sample_rate = 16000;
  impl_->config.keywords_threshold = impl_->threshold;

  if (!impl_->keywords_file.empty()) {
    impl_->config.keywords_file = impl_->keywords_file;
  }

  auto spotter = sherpa_onnx::cxx::KeywordSpotter::Create(impl_->config);
  if (!spotter.Get()) {
    std::cerr << "[real_backend] real_sherpa_wakeword load failed: "
              << "KeywordSpotter::Create returned null (model_dir="
              << impl_->model_dir << ")" << std::endl;
    return;
  }

  impl_->spotter =
    std::make_unique<sherpa_onnx::cxx::KeywordSpotter>(std::move(spotter));
  auto stream = impl_->spotter->CreateStream();
  impl_->stream =
    std::make_unique<sherpa_onnx::cxx::OnlineStream>(std::move(stream));

  impl_->is_loaded = true;
}

void RealSherpaWakewordBackend::unload()
{
  impl_->stream.reset();
  impl_->spotter.reset();
  impl_->is_loaded = false;
}

bool RealSherpaWakewordBackend::loaded() const
{
  return impl_->is_loaded;
}

bool RealSherpaWakewordBackend::detect(
  const int16_t * pcm, size_t samples,
  float & confidence, std::string & keyword)
{
  if (!impl_->is_loaded || pcm == nullptr || samples == 0) {
    return false;
  }

  // Convert int16 to float32 (normalize to [-1, 1])
  std::vector<float> float_samples(samples);
  for (size_t i = 0; i < samples; ++i) {
    float_samples[i] = static_cast<float>(pcm[i]) / 32768.0f;
  }

  impl_->stream->AcceptWaveform(16000, float_samples.data(),
                                 static_cast<int32_t>(samples));

  while (impl_->spotter->IsReady(impl_->stream.get())) {
    impl_->spotter->Decode(impl_->stream.get());
  }

  auto result = impl_->spotter->GetResult(impl_->stream.get());

  if (!result.keyword.empty()) {
    confidence = 1.0f;
    keyword = result.keyword;
    // Reset stream after detection for next utterance
    impl_->spotter->Reset(impl_->stream.get());
    return true;
  }

  return false;
}

void RealSherpaWakewordBackend::reset()
{
  if (impl_->is_loaded && impl_->spotter && impl_->stream) {
    impl_->spotter->Reset(impl_->stream.get());
  }
}

}  // namespace k1muse_ai_runtime

#else  // !K1MUSE_ENABLE_REAL_K1_BACKENDS

namespace k1muse_ai_runtime
{

// Stub when real backends are not enabled.
struct RealSherpaWakewordBackend::Impl {};

RealSherpaWakewordBackend::RealSherpaWakewordBackend(
  const std::string &, float, const std::string &) {}

RealSherpaWakewordBackend::~RealSherpaWakewordBackend() = default;

const std::string & RealSherpaWakewordBackend::name() const
{
  static const std::string n{"real_sherpa_wakeword(stub)"};
  return n;
}

void RealSherpaWakewordBackend::load() {}
void RealSherpaWakewordBackend::unload() {}
bool RealSherpaWakewordBackend::loaded() const { return false; }
bool RealSherpaWakewordBackend::detect(
  const int16_t *, size_t, float &, std::string &) { return false; }
void RealSherpaWakewordBackend::reset() {}

}  // namespace k1muse_ai_runtime

#endif  // K1MUSE_ENABLE_REAL_K1_BACKENDS
