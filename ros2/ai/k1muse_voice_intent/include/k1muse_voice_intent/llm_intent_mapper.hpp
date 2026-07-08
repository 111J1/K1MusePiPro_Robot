#pragma once

#include <string>

#include "k1muse_voice_intent/intent_types.hpp"
#include "k1muse_voice_intent/llm_response_validator.hpp"

namespace k1muse_voice_intent {

// Deterministic mapper from validated LLM JSON output to IntentDecision.
//
// This is a pure function — no I/O, no ROS.  It receives the parsed
// fields from LlmValidationResult and applies the mapping table
// defined in PHASE2_LLAMA_INTEGRATION.md §8.1.
//
// The mapper enforces:
//  - kind → intent_name / action / target mapping
//  - direction / target constraints per kind
//  - requires_confirmation for shutdown/reboot
//  - TTS reply generation from local templates
//
// Returns IntentDecision with matched=false if the combination is
// illegal (e.g. "find" with empty target).

IntentDecision MapLlmOutput(const LlmValidationResult& validated);

}  // namespace k1muse_voice_intent
