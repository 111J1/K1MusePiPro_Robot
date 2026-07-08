#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

namespace k1muse_voice_intent {

struct IntentDecision {
  bool matched = false;
  std::string intent_name;   // was intent_type
  std::string action;        // was command
  std::string target;
  std::string location;      // new field
  std::string value;
  float confidence = 0.0f;
  bool requires_confirmation = false;  // new field
  std::string tts_reply;
  float distance_m = 0.0f;   // motion distance (for move), default picked by TaskManager
  float angle_rad = 0.0f;    // rotation angle (for rotate), default picked by TaskManager
};

// ─────────────────────────────────────────────────────────────
// Upgraded LLM client contract (§7 of PHASE2_LLAMA_INTEGRATION.md)
// ─────────────────────────────────────────────────────────────

enum class LlmStatus {
  kOk,
  kCancelled,
  kTimeout,
  kUnavailable,
  kHttpError,
  kResponseTooLarge,
  kInvalidResponse,
  kInternalError,
};

struct LlmRequestContext {
  std::string request_id;
  uint64_t epoch;
  std::chrono::steady_clock::time_point deadline;
  std::shared_ptr<std::atomic_bool> cancelled;

  // Factory for a context that never expires (used by mock / tests).
  static LlmRequestContext NeverExpires() {
    LlmRequestContext ctx;
    ctx.request_id = "never";
    ctx.epoch = 0;
    ctx.deadline = std::chrono::steady_clock::time_point::max();
    ctx.cancelled = std::make_shared<std::atomic_bool>(false);
    return ctx;
  }
};

struct LlmResult {
  LlmStatus status = LlmStatus::kInternalError;
  std::string content;       // Raw text output from model (validated JSON)
  int http_status{0};
  int64_t latency_ms{0};
  std::string error;         // Human-readable error when status != kOk
};

}  // namespace k1muse_voice_intent
