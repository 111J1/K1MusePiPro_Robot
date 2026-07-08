#pragma once

#include "k1muse_ai_runtime/backends/wakeword_backend.hpp"

#include <string>

namespace k1muse_ai_runtime
{

/// Deterministic mock wakeword backend for testing.
/// Detection triggers when any sample in the chunk equals \p trigger_sample_value
/// (default 0x7FFF, i.e. maximum positive int16).
class MockWakewordBackend : public WakewordBackend
{
public:
  MockWakewordBackend();

  const std::string & name() const override;

  void load() override;
  void unload() override;
  bool loaded() const override;

  bool detect(
    const int16_t * pcm, size_t samples,
    float & confidence, std::string & keyword) override;

  void reset() override;

  // Configuration setters (for test customization)
  void set_trigger_sample_value(int16_t value);
  void set_confidence(float confidence);
  void set_keyword(const std::string & keyword);

private:
  bool loaded_{false};
  int16_t trigger_sample_value_{0x7FFF};
  float confidence_{1.0f};
  std::string keyword_{"hello"};
};

}  // namespace k1muse_ai_runtime
