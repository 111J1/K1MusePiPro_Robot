#pragma once

#include <string>

#include "k1muse_voice_intent/intent_types.hpp"

namespace k1muse_voice_intent {

enum class FastIntentCategory {
  kNone,
  kSafetyFast,
  kSimpleActionFast,
  kQueryFast,
  kChatFast,
  kExtractiveFast,
  kSemanticLlm,
  kRecoverableUnknown,
};

enum class SlotState {
  kNoSlotNeeded,
  kSlotValid,
  kSlotMissing,
  kSlotAmbiguous,
};

struct FastIntentCandidate {
  IntentDecision decision;
  FastIntentCategory category = FastIntentCategory::kNone;
  SlotState slot_state = SlotState::kNoSlotNeeded;
  std::string route_reason;
};

FastIntentCandidate MatchFastIntentCandidate(const std::string& text);
IntentDecision MatchFastIntent(const std::string& text);

const char* ToString(FastIntentCategory category);
const char* ToString(SlotState slot_state);

}  // namespace k1muse_voice_intent
