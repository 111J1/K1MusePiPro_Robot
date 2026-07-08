#pragma once

#include <string>

namespace k1muse_voice_intent {

// Strict JSON validator for LLM chat-completion output.
//
// Validates that the response is exactly ONE JSON object with only
// the allowed fields (kind, direction, target, reply) and value constraints.
// This is a pure function with no dependencies — testable offline.
//
// See PHASE2_LLAMA_INTEGRATION.md §8 for the output contract.

struct LlmValidationResult {
  bool valid = false;
  std::string error;  // Human-readable error when !valid

  // Parsed fields — only meaningful when valid == true.
  std::string kind;
  std::string direction;
  std::string target;
  std::string reply;
};

LlmValidationResult ValidateLlmResponse(const std::string& raw_text);

}  // namespace k1muse_voice_intent
