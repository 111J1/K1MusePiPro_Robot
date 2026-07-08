#pragma once

#include <string>

#include "k1muse_voice_intent/intent_types.hpp"

namespace k1muse_voice_intent {

// Abstract interface for LLM-based intent recognition clients.
//
// Upgraded in Phase 2 (llama-server real integration) to include
// deadline, cancel and rich error classification.  Mock and real
// clients MUST implement the same contract (§7 of PHASE2 plan).
class LlmIntentClient {
 public:
  virtual ~LlmIntentClient() = default;

  // Returns true if the LLM backend is reachable and healthy.
  // `reason` is set when returning false (e.g. "connection refused",
  // "model loading").
  virtual bool health_check(std::string* reason = nullptr) = 0;

  // Send text to the LLM for intent extraction.
  //
  // The request context carries deadline, cancel flag and identity.
  // Implementations MUST observe the cancel flag during blocking I/O
  // and honour the deadline.
  //
  // On success the `content` field holds the raw validated JSON
  // output from the model.  Callers pipe it through
  // LlmIntentMapper to obtain the final IntentDecision.
  virtual LlmResult complete_intent(
      const std::string& text,
      const LlmRequestContext& context) = 0;

  // Prime the LLM backend's KV cache with the system prompt.
  // Should be called once before processing user requests.
  // Returns true on success.  `reason` is set when returning false.
  virtual bool warmup(std::string* reason = nullptr) = 0;

  // Human-readable name of this client (e.g. "mock", "real").
  virtual const std::string& name() const = 0;
};

}  // namespace k1muse_voice_intent
