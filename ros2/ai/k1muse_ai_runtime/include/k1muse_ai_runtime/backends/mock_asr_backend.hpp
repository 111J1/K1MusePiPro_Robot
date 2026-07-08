#pragma once

#include "k1muse_ai_runtime/backends/asr_backend.hpp"

#include <string>

namespace k1muse_ai_runtime
{

/// Deterministic mock ASR backend for testing.
/// Returns the configured text/confidence/language for any non-empty segment.
/// Returns failure with reason "empty segment" when PCM is empty.
class MockAsrBackend : public AsrBackend
{
public:
  MockAsrBackend();

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  Result transcribe(const std::vector<int16_t> & pcm, uint32_t sample_rate) override;

  void reset() override;

  // Configuration setters (for test customization)
  void set_mock_text(const std::string & text);
  void set_mock_confidence(float confidence);
  void set_mock_language(const std::string & language);

private:
  bool loaded_{false};
  std::string mock_text_{"mock transcription"};
  float mock_confidence_{0.95f};
  std::string mock_language_{"zh"};
};

}  // namespace k1muse_ai_runtime
