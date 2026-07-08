#include "k1muse_voice_intent/intent_router.hpp"

#include <string>
#include <utility>

#include "k1muse_voice_intent/asr_text_cleaner.hpp"
#include "k1muse_voice_intent/fast_intent.hpp"
#include "k1muse_voice_intent/llm_intent_mapper.hpp"

namespace k1muse_voice_intent {
namespace {

int Utf8CharCount(const std::string& s) {
  int count = 0;
  for (size_t i = 0; i < s.size(); ++count) {
    unsigned char c = static_cast<unsigned char>(s[i]);
    if (c < 0x80) {
      i += 1;
    } else if (c < 0xE0) {
      i += 2;
    } else if (c < 0xF0) {
      i += 3;
    } else {
      i += 4;
    }
  }
  return count;
}

void CopyFastDiagnostics(const FastIntentCandidate& candidate,
                         RouterResult* result) {
  result->fast_category = ToString(candidate.category);
  result->fast_slot_state = ToString(candidate.slot_state);
  result->fast_route_reason = candidate.route_reason;
}

RouterResult MakeSafeFallbackResult(const std::string& reason) {
  RouterResult result;
  result.failed = false;
  result.failure_reason = reason;
  result.route_source = "safe_fallback";
  result.decision.matched = true;
  result.decision.intent_name = "unknown";
  result.decision.action = "ask_repeat";
  result.decision.confidence = 0.2f;
  result.decision.tts_reply = "我还没理解，请换一种说法。";
  return result;
}

bool IsSafeFastFallbackCandidate(const FastIntentCandidate& candidate) {
  if (!candidate.decision.matched) {
    return false;
  }
  switch (candidate.category) {
    case FastIntentCategory::kSafetyFast:
    case FastIntentCategory::kQueryFast:
    case FastIntentCategory::kChatFast:
      return true;
    case FastIntentCategory::kExtractiveFast:
      return candidate.decision.intent_name == "reminder" &&
             candidate.decision.action == "list";
    default:
      return false;
  }
}

bool ShouldAcceptFastCandidate(const FastIntentCandidate& candidate,
                               const RouterConfig& config,
                               int text_chars,
                               std::string* reject_reason) {
  if (!candidate.decision.matched) {
    if (reject_reason) {
      *reject_reason = candidate.slot_state == SlotState::kSlotMissing
        ? "slot_missing" : "no_fast_match";
    }
    return false;
  }

  if (candidate.slot_state == SlotState::kSlotMissing) {
    if (reject_reason) *reject_reason = "slot_missing";
    return false;
  }
  if (candidate.slot_state == SlotState::kSlotAmbiguous) {
    if (reject_reason) *reject_reason = "slot_ambiguous";
    return false;
  }

  switch (candidate.category) {
    case FastIntentCategory::kSafetyFast:
      if (text_chars <= config.fast_stop_max_chars) return true;
      if (reject_reason) *reject_reason = "safety_text_too_long";
      return false;

    case FastIntentCategory::kSimpleActionFast:
      if (text_chars <= config.fast_simple_action_max_chars) return true;
      if (reject_reason) *reject_reason = "simple_action_too_complex";
      return false;

    case FastIntentCategory::kQueryFast:
    case FastIntentCategory::kChatFast:
      if (candidate.decision.tts_reply.empty()) {
        if (reject_reason) *reject_reason = "local_reply_missing";
        return false;
      }
      if (text_chars <= config.fast_query_chat_max_chars) return true;
      if (reject_reason) *reject_reason = "query_chat_too_complex";
      return false;

    case FastIntentCategory::kExtractiveFast:
      if (candidate.decision.action == "find") {
        if (text_chars <= config.fast_extractive_find_max_chars) return true;
        if (reject_reason) *reject_reason = "find_text_too_complex";
        return false;
      }
      if (candidate.decision.intent_name == "reminder") {
        if (text_chars <= config.fast_extractive_reminder_max_chars) return true;
        if (reject_reason) *reject_reason = "reminder_text_too_complex";
        return false;
      }
      if (reject_reason) *reject_reason = "extractive_action_unknown";
      return false;

    default:
      if (reject_reason) *reject_reason = "category_not_fast";
      return false;
  }
}

RouterResult MakeSafeFastFallbackResult(const FastIntentCandidate& candidate,
                                        const std::string& reason) {
  RouterResult result;
  result.failed = false;
  result.failure_reason = reason;
  result.decision = candidate.decision;
  result.from_fast_intent = true;
  result.route_source = "safe_fast_after_llm_invalid";
  CopyFastDiagnostics(candidate, &result);
  return result;
}

}  // namespace

IntentRouter::IntentRouter(RouterConfig config,
                           std::unique_ptr<LlmIntentClient> llm_client)
    : config_(std::move(config)), llm_client_(std::move(llm_client)) {}

RouterResult IntentRouter::process(const std::string& raw_text,
                                   const LlmRequestContext* llm_context) {
  RouterResult result;

  std::string cleaned = CleanAsrText(raw_text);

  if (cleaned.empty()) {
    result.failed = true;
    result.failure_reason = "empty text";
    result.route_source = "failed";
    return result;
  }

  FastIntentCandidate fast_candidate;
  bool fast_candidate_seen = false;
  const int text_chars = Utf8CharCount(cleaned);

  if (config_.allow_fast_intent) {
    fast_candidate = MatchFastIntentCandidate(cleaned);
    fast_candidate_seen = fast_candidate.decision.matched ||
      fast_candidate.category != FastIntentCategory::kNone;

    if (fast_candidate_seen) {
      CopyFastDiagnostics(fast_candidate, &result);

      if (!fast_candidate.decision.matched &&
          fast_candidate.slot_state == SlotState::kSlotMissing) {
        result = MakeSafeFallbackResult("fast_slot_missing: " + fast_candidate.route_reason);
        CopyFastDiagnostics(fast_candidate, &result);
        return result;
      }

      std::string reject_reason;
      if (ShouldAcceptFastCandidate(fast_candidate, config_, text_chars,
                                    &reject_reason)) {
        result.decision = fast_candidate.decision;
        result.from_fast_intent = true;
        result.route_source = "fast";
        return result;
      }
      result.fast_reject_reason = reject_reason;
    }
  }

  if (config_.llm_fallback_enabled && llm_client_) {
    result.llm_called = true;
    result.route_source = "llm";

    LlmRequestContext ctx;
    if (llm_context) {
      ctx = *llm_context;
    } else {
      ctx = LlmRequestContext::NeverExpires();
    }

    LlmResult llm_result = llm_client_->complete_intent(cleaned, ctx);

    if (llm_result.status == LlmStatus::kOk) {
      result.llm_status = "ok";
      result.llm_raw_json = llm_result.content;

      LlmValidationResult validated = ValidateLlmResponse(llm_result.content);
      if (validated.valid) {
        IntentDecision mapped = MapLlmOutput(validated);
        if (mapped.matched) {
          if (result.fast_reject_reason == "simple_action_too_complex" &&
              mapped.intent_name == "action" &&
              (mapped.action == "move" || mapped.action == "rotate" ||
               mapped.action == "lift")) {
            RouterResult fallback = MakeSafeFallbackResult(
                "complex_action_not_executed: " + mapped.action);
            fallback.llm_called = true;
            fallback.llm_status = "ok_rejected_by_policy";
            fallback.llm_raw_json = llm_result.content;
            fallback.fast_reject_reason = result.fast_reject_reason;
            CopyFastDiagnostics(fast_candidate, &fallback);
            return fallback;
          }
          result.decision = mapped;
          result.route_source = "llm";
          return result;
        }
      }

      if (config_.fast_allow_safe_fallback_after_llm_invalid &&
          IsSafeFastFallbackCandidate(fast_candidate)) {
        RouterResult fallback = MakeSafeFastFallbackResult(
            fast_candidate,
            validated.error.empty() ? "llm_mapping_failed; using safe fast fallback"
                                    : "llm_invalid_response: " + validated.error +
                                      "; using safe fast fallback");
        fallback.llm_called = true;
        fallback.llm_status = "invalid_response";
        fallback.llm_raw_json = llm_result.content;
        fallback.fast_reject_reason = result.fast_reject_reason;
        return fallback;
      }

      RouterResult fallback = MakeSafeFallbackResult(
          validated.error.empty() ? "llm_mapping_failed"
                                  : "llm_invalid_response: " + validated.error);
      fallback.llm_called = true;
      fallback.llm_status = "invalid_response";
      fallback.llm_raw_json = llm_result.content;
      fallback.fast_reject_reason = result.fast_reject_reason;
      CopyFastDiagnostics(fast_candidate, &fallback);
      return fallback;
    }

    if (llm_result.status == LlmStatus::kInvalidResponse) {
      if (config_.fast_allow_safe_fallback_after_llm_invalid &&
          IsSafeFastFallbackCandidate(fast_candidate)) {
        RouterResult fallback = MakeSafeFastFallbackResult(
            fast_candidate,
            "llm_invalid_response: " + llm_result.error +
            "; using safe fast fallback");
        fallback.llm_called = true;
        fallback.llm_status = "invalid_response";
        fallback.fast_reject_reason = result.fast_reject_reason;
        return fallback;
      }

      RouterResult fallback = MakeSafeFallbackResult(
          "llm_invalid_response: " + llm_result.error);
      fallback.llm_called = true;
      fallback.llm_status = "invalid_response";
      fallback.fast_reject_reason = result.fast_reject_reason;
      CopyFastDiagnostics(fast_candidate, &fallback);
      return fallback;
    }

    switch (llm_result.status) {
      case LlmStatus::kCancelled:
        result.failed = true;
        result.failure_reason = "cancelled";
        result.llm_status = "cancelled";
        break;
      case LlmStatus::kTimeout:
        result.failed = true;
        result.failure_reason = "llm_timeout";
        result.llm_status = "timeout";
        break;
      case LlmStatus::kUnavailable:
        result.failed = true;
        result.failure_reason = "llm_unavailable";
        result.llm_status = "unavailable";
        break;
      case LlmStatus::kHttpError:
        result.failed = true;
        result.failure_reason = "llm_http_error: " + llm_result.error;
        result.llm_status = "http_error";
        break;
      case LlmStatus::kResponseTooLarge:
        result.failed = true;
        result.failure_reason = "llm_response_too_large";
        result.llm_status = "response_too_large";
        break;
      default:
        result.failed = true;
        result.failure_reason = "llm_error: " + llm_result.error;
        result.llm_status = "error";
        break;
    }
    return result;
  }

  result.failed = true;
  result.failure_reason = "no intent matched";
  result.route_source = "failed";
  result.fast_reject_reason = result.fast_reject_reason.empty()
    ? "llm_disabled_or_unavailable" : result.fast_reject_reason;
  return result;
}

void IntentRouter::to_intent_lite(const RouterResult& result,
                                  const std::string& trace_id,
                                  const std::string& request_id,
                                  const std::string& utterance_id,
                                  uint64_t epoch, IntentLiteFields& out) {
  out.intent_name = result.decision.intent_name;
  out.target = result.decision.target;
  out.action = result.decision.action;
  out.location = result.decision.location;
  out.value = result.decision.value;
  out.confidence = result.decision.confidence;
  out.requires_confirmation = result.decision.requires_confirmation;
  out.distance_m = result.decision.distance_m;
  out.angle_rad = result.decision.angle_rad;
  (void)trace_id;
  (void)request_id;
  (void)utterance_id;
  (void)epoch;
}

bool IntentRouter::to_tts_request(const RouterResult& result,
                                  const std::string& trace_id,
                                  const std::string& request_id,
                                  uint64_t epoch, TtsRequestFields& out) {
  if (result.failed || result.decision.tts_reply.empty()) {
    return false;
  }

  out.source = "intent_router";
  out.priority = 1;  // PRIORITY_USER_REPLY
  out.text = result.decision.tts_reply;
  out.voice = "";
  (void)trace_id;
  (void)request_id;
  (void)epoch;
  return true;
}

}  // namespace k1muse_voice_intent
