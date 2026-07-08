#include "k1muse_voice_intent/real_llm_intent_client.hpp"

#include "k1muse_voice_intent/llm_response_validator.hpp"

namespace k1muse_voice_intent {

RealLlmIntentClient::RealLlmIntentClient(LlamaServerClient::Config config)
    : name_("real"), transport_(std::move(config)) {}

RealLlmIntentClient::~RealLlmIntentClient() = default;

bool RealLlmIntentClient::health_check(std::string* reason) {
  return transport_.health_check(reason);
}

bool RealLlmIntentClient::warmup(std::string* reason) {
  return transport_.warmup(reason);
}

LlmResult RealLlmIntentClient::complete_intent(
    const std::string& text, const LlmRequestContext& context) {
  // Step 1 — HTTP transport.
  LlmResult result = transport_.complete(text, context);
  if (result.status != LlmStatus::kOk) {
    return result;  // Transport error already set.
  }

  // Step 2 — Strict JSON validation.
  LlmValidationResult validated = ValidateLlmResponse(result.content);
  if (!validated.valid) {
    result.status = LlmStatus::kInvalidResponse;
    result.error = validated.error;
    result.content.clear();
    return result;
  }

  // Step 3 — Return the validated raw JSON.
  // The caller (IntentRouter) will pass it through LlmIntentMapper
  // to obtain the final IntentDecision.
  // `result.content` already contains the raw JSON string.
  return result;
}

const std::string& RealLlmIntentClient::name() const { return name_; }

LlamaServerClient& RealLlmIntentClient::transport() { return transport_; }

}  // namespace k1muse_voice_intent
