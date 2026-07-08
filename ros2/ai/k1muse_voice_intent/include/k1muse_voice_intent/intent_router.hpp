#pragma once

#include <memory>
#include <string>

#include "k1muse_voice_intent/intent_types.hpp"
#include "k1muse_voice_intent/llm_intent_client.hpp"

namespace k1muse_voice_intent {

struct RouterConfig {
  bool allow_fast_intent = true;
  bool llm_fallback_enabled = true;
  int complex_text_chars = 10;  // Deprecated compatibility limit.
  int max_tts_chars = 200;

  int fast_stop_max_chars = 999;
  int fast_simple_action_max_chars = 8;
  int fast_query_chat_max_chars = 12;
  int fast_extractive_find_max_chars = 32;
  int fast_extractive_reminder_max_chars = 48;
  bool fast_allow_safe_fallback_after_llm_invalid = true;
};

struct RouterResult {
  IntentDecision decision;
  bool from_fast_intent = false;
  bool llm_called = false;
  bool failed = false;
  std::string failure_reason;
  // When llm_called and not failed, the raw validated JSON from the model.
  std::string llm_raw_json;
  std::string route_source;
  std::string fast_category;
  std::string fast_slot_state;
  std::string fast_route_reason;
  std::string fast_reject_reason;
  std::string llm_status;
};

// Output fields matching IntentLite.msg (no ROS dependency).
struct IntentLiteFields {
  std::string intent_name;
  std::string target;
  std::string action;
  std::string location;
  std::string value;
  float confidence = 0.0f;
  bool requires_confirmation = false;
  float distance_m = 0.0f;
  float angle_rad = 0.0f;
};

// Output fields matching TtsTextRequest.msg (no ROS dependency).
struct TtsRequestFields {
  std::string source;
  uint8_t priority = 0;  // PRIORITY_USER_REPLY = 1
  std::string text;
  std::string voice;
};

class IntentRouter {
 public:
  IntentRouter(RouterConfig config, std::unique_ptr<LlmIntentClient> llm_client);

  // Process raw ASR text. Returns RouterResult.
  //
  // When LLM fallback is invoked, the LlmRequestContext carries
  // cancel/deadline semantics.  The LLM output is validated and
  // mapped through LlmIntentMapper.
  RouterResult process(const std::string& raw_text,
                       const LlmRequestContext* llm_context = nullptr);

  // Access the underlying LLM client (for health checks, etc.).
  LlmIntentClient* client() const { return llm_client_.get(); }

  // Map RouterResult to IntentLite output fields.
  static void to_intent_lite(const RouterResult& result,
                             const std::string& trace_id,
                             const std::string& request_id,
                             const std::string& utterance_id, uint64_t epoch,
                             IntentLiteFields& out);

  // Map RouterResult to TTS request fields. Returns true if a reply is needed.
  static bool to_tts_request(const RouterResult& result,
                             const std::string& trace_id,
                             const std::string& request_id, uint64_t epoch,
                             TtsRequestFields& out);

 private:
  RouterConfig config_;
  std::unique_ptr<LlmIntentClient> llm_client_;
};

}  // namespace k1muse_voice_intent
