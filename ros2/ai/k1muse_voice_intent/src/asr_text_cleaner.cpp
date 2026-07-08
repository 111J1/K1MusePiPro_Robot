#include "k1muse_voice_intent/asr_text_cleaner.hpp"

#include <algorithm>
#include <regex>

namespace k1muse_voice_intent {

std::string CleanAsrText(const std::string& text) {
  // Strip <|...|> tags (SenseVoice emotion/language/event tags).
  static const std::regex tag_re("<\\|[^|]*\\|>");
  std::string cleaned = std::regex_replace(text, tag_re, "");

  // Normalize whitespace: collapse runs of spaces/tabs into a single space.
  static const std::regex ws_re("[ \\t]+");
  cleaned = std::regex_replace(cleaned, ws_re, " ");

  // Trim leading and trailing whitespace.
  const auto first = cleaned.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = cleaned.find_last_not_of(" \t\r\n");
  return cleaned.substr(first, last - first + 1);
}

bool IsMeaningfulText(const std::string& text) {
  return !text.empty();
}

}  // namespace k1muse_voice_intent
