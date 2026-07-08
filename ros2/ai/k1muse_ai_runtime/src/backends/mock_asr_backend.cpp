#include "k1muse_ai_runtime/backends/mock_asr_backend.hpp"

namespace k1muse_ai_runtime
{

MockAsrBackend::MockAsrBackend() = default;

const std::string & MockAsrBackend::name() const
{
  static const std::string n{"mock_asr"};
  return n;
}

void MockAsrBackend::load()
{
  loaded_ = true;
}

void MockAsrBackend::unload()
{
  loaded_ = false;
}

bool MockAsrBackend::loaded() const
{
  return loaded_;
}

AsrBackend::Result MockAsrBackend::transcribe(
  const std::vector<int16_t> & pcm, uint32_t /*sample_rate*/)
{
  if (!loaded_) {
    return Result{false, "", 0.0f, "", "backend not loaded"};
  }

  if (pcm.empty()) {
    return Result{false, "", 0.0f, "", "empty segment"};
  }

  return Result{true, mock_text_, mock_confidence_, mock_language_, ""};
}

void MockAsrBackend::reset()
{
  // Nothing to reset beyond load state for the mock.
}

void MockAsrBackend::set_mock_text(const std::string & text)
{
  mock_text_ = text;
}

void MockAsrBackend::set_mock_confidence(float confidence)
{
  mock_confidence_ = confidence;
}

void MockAsrBackend::set_mock_language(const std::string & language)
{
  mock_language_ = language;
}

}  // namespace k1muse_ai_runtime
