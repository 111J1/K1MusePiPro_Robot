#pragma once

#include <string>
#include <unordered_map>

#include "k1muse_voice_intent/intent_types.hpp"
#include "k1muse_voice_intent/llm_intent_client.hpp"

namespace k1muse_voice_intent {

// Configurable mock LLM intent client for testing.
//
// Upgraded to the Phase 2 LlmIntentClient contract (returns LlmResult
// instead of IntentDecision).  The mock stores JSON content strings
// so the full validator→mapper pipeline can be exercised.
//
// Default behaviour:
//   - complete_intent("未知") or empty text -> LlmStatus::kInvalidResponse
//   - Otherwise returns a configurable default JSON content string
//
// Use set_json_override() to add custom text→JSON mappings.
// Use set_default_json() to change the fallback response.
class MockLlmIntentClient : public LlmIntentClient {
 public:
  MockLlmIntentClient();

  // LlmIntentClient interface
  bool health_check(std::string* reason = nullptr) override;
  bool warmup(std::string* reason = nullptr) override;
  LlmResult complete_intent(
      const std::string& text,
      const LlmRequestContext& context) override;
  const std::string& name() const override;

  // --- Test configuration ---

  // Add a pattern override: if text contains `pattern`, return
  // `llm_json` as the content string inside a kOk result.
  void set_json_override(const std::string& pattern,
                         const std::string& llm_json);

  // Set the default JSON content returned when no pattern matches
  // and the text is not considered "unmatched".
  void set_default_json(const std::string& llm_json);

  // Control whether health_check() returns true or false.
  void set_healthy(bool healthy);

  // If true, complete_intent() returns kTimeout instead of the
  // normal result (for testing timeout handling).
  void set_force_timeout(bool force);

  // If true, complete_intent() returns kUnavailable.
  void set_force_unavailable(bool force);

  // If non-zero, adds artificial latency (ms).
  void set_simulated_latency_ms(int64_t ms);

  // Get the number of times complete_intent() was called.
  int call_count() const;

 private:
  std::string name_;
  bool healthy_ = true;
  int call_count_ = 0;
  std::string default_json_;
  std::unordered_map<std::string, std::string> json_overrides_;
  bool force_timeout_ = false;
  bool force_unavailable_ = false;
  int64_t simulated_latency_ms_ = 0;
};

}  // namespace k1muse_voice_intent
