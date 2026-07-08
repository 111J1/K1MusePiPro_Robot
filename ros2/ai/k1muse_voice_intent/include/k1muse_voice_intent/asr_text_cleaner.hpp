#pragma once

#include <string>

namespace k1muse_voice_intent {

// Strip ASR tags like <|...|>, normalize whitespace, and trim.
std::string CleanAsrText(const std::string& text);

// Returns true if the text is non-empty after cleaning.
bool IsMeaningfulText(const std::string& text);

}  // namespace k1muse_voice_intent
