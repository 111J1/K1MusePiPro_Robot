#pragma once

#include "k1muse_ai_runtime/backends/tts_backend.hpp"

#include <string>

namespace k1muse_ai_runtime
{

/// Deterministic mock TTS backend for testing.
/// Generates a fixed PCM pattern (440 Hz sine wave) at the configured sample
/// rate for the configured duration. Returns failure with reason "empty text"
/// when the text argument is empty.
class MockTtsBackend : public TtsBackend
{
public:
  MockTtsBackend();

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  void warmup() override;

  Result synthesize(const std::string & text, const std::string & voice) override;

  void reset() override;

  // Configuration setters (for test customization)
  void set_mock_sample_rate(uint32_t sample_rate);
  void set_mock_channels(uint8_t channels);
  void set_mock_encoding(const std::string & encoding);
  void set_mock_duration_ms(uint32_t duration_ms);

private:
  bool loaded_{false};
  uint32_t mock_sample_rate_{16000};
  uint8_t mock_channels_{1};
  std::string mock_encoding_{"s16le"};
  uint32_t mock_duration_ms_{500};
};

}  // namespace k1muse_ai_runtime
