#include "k1muse_voice_intent/llm_response_validator.hpp"

#include <cctype>
#include <string>
#include <unordered_set>

namespace k1muse_voice_intent {

namespace {

// ── Allowed values ──

const std::unordered_set<std::string> kAllowedKinds = {
    "move",        "stop",         "rotate",         "lift",
    "find",        "query_introduce", "query_time",   "reminder_create",
    "reminder_list", "system_shutdown", "system_reboot",
    "chat",         "ask_repeat",  "unknown",
};

const std::unordered_set<std::string> kAllowedDirections = {
    "forward", "backward", "left", "right", "up", "down",
    "none",  // LLM may output "none" for queries without direction
};

// ── Helpers ──

// Strip leading/trailing whitespace.
std::string Strip(const std::string& s) {
  size_t a = 0;
  while (a < s.size() && (s[a] == ' ' || s[a] == '\t' ||
                           s[a] == '\n' || s[a] == '\r'))
    ++a;
  size_t b = s.size();
  while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' ||
                    s[b - 1] == '\n' || s[b - 1] == '\r'))
    --b;
  return s.substr(a, b - a);
}

// Remove surrounding ``` fences (Markdown code blocks).
std::string StripFences(const std::string& s) {
  std::string t = Strip(s);
  if (t.size() >= 6 && t.substr(0, 3) == "```") {
    size_t end = t.find('\n', 3);
    if (end != std::string::npos) {
      t = t.substr(end + 1);
    }
  }
  if (t.size() >= 3 && t.substr(t.size() - 3) == "```") {
    t = t.substr(0, t.size() - 3);
  }
  return Strip(t);
}

// Extract the first balanced { … } JSON object.
// Returns empty string if none found.
std::string ExtractFirstJsonObject(const std::string& s) {
  size_t start = s.find('{');
  if (start == std::string::npos) return {};
  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (size_t i = start; i < s.size(); ++i) {
    char c = s[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (in_string) {
      if (c == '\\') escaped = true;
      if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') {
      in_string = true;
      continue;
    }
    if (c == '{') ++depth;
    if (c == '}') {
      --depth;
      if (depth == 0) {
        return s.substr(start, i - start + 1);
      }
    }
  }
  return {};  // Unbalanced
}

// Unescape a JSON string fragment (handles \\, \", \n, \r, \t, \/).
std::string Unescape(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\\' && i + 1 < s.size()) {
      switch (s[i + 1]) {
        case '"':  out += '"';  break;
        case '\\': out += '\\'; break;
        case '/':  out += '/';  break;
        case 'n':  out += '\n'; break;
        case 'r':  out += '\r'; break;
        case 't':  out += '\t'; break;
        case 'u': {
          // Skip \uXXXX — we don't need full unicode support for
          // allowed field values, just preserve as-is.
          if (i + 5 < s.size()) {
            out += s.substr(i, 6);
            i += 5;
          } else {
            out += s[i];
          }
          break;
        }
        default: out += s[i + 1]; break;
      }
      ++i;
    } else {
      out += s[i];
    }
  }
  return out;
}

// Parse a single JSON string value from `s` starting at position `pos`.
// Returns the unescaped string and advances `pos` past the closing ".
// On failure, returns empty string and does not advance `pos`.
std::string ParseJsonString(const std::string& s, size_t& pos) {
  while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos])))
    ++pos;
  if (pos >= s.size() || s[pos] != '"') return {};
  ++pos;  // skip opening "
  std::string raw;
  bool escaped = false;
  while (pos < s.size()) {
    char c = s[pos];
    if (escaped) {
      raw += c;
      escaped = false;
      ++pos;
      continue;
    }
    if (c == '\\') {
      raw += c;
      escaped = true;
      ++pos;
      continue;
    }
    if (c == '"') {
      ++pos;  // skip closing "
      return Unescape(raw);
    }
    raw += c;
    ++pos;
  }
  return {};  // Unterminated string
}

// Check that `pos` is at end or only whitespace/commas remain.
bool AtEnd(const std::string& s, size_t pos) {
  while (pos < s.size()) {
    char c = s[pos];
    if (c != ' ' && c != '\t' && c != '\n' && c != '\r') return false;
    ++pos;
  }
  return true;
}

// Check for control characters (U+0000–U+001F except common whitespace).
bool HasControlChars(const std::string& s) {
  for (char c : s) {
    unsigned char uc = static_cast<unsigned char>(c);
    if (uc < 0x20 && c != '\n' && c != '\r' && c != '\t') return true;
  }
  return false;
}

}  // namespace

LlmValidationResult ValidateLlmResponse(const std::string& raw_text) {
  LlmValidationResult result;

  // Step 1 — Strip fences and extract first JSON object.
  std::string json = StripFences(raw_text);
  json = ExtractFirstJsonObject(json);
  if (json.empty()) {
    result.error = "no JSON object found in response";
    return result;
  }

  // Step 2 — Manual key-value parser for the restricted schema.
  // Expected: { "kind": "...", "direction": "...", "target": "...", "reply": "..." }
  // Only these four keys are allowed; extra keys cause rejection.

  bool has_kind = false, has_direction = false, has_target = false, has_reply = false;
  int key_count = 0;

  size_t pos = 0;
  // Skip opening {
  while (pos < json.size() && json[pos] != '{') ++pos;
  if (pos >= json.size()) { result.error = "missing {"; return result; }
  ++pos;

  while (pos < json.size()) {
    // Skip whitespace
    while (pos < json.size() &&
           std::isspace(static_cast<unsigned char>(json[pos])))
      ++pos;
    if (pos >= json.size()) break;

    // Check for closing }
    if (json[pos] == '}') {
      ++pos;
      if (!AtEnd(json, pos)) {
        result.error = "trailing content after closing }";
        return result;
      }
      break;
    }

    // Expect comma between pairs (skip first iteration)
    if (key_count > 0) {
      if (json[pos] != ',') {
        result.error = "expected comma between fields";
        return result;
      }
      ++pos;
      while (pos < json.size() &&
             std::isspace(static_cast<unsigned char>(json[pos])))
        ++pos;
    }

    // Parse key (must be a JSON string)
    std::string key = ParseJsonString(json, pos);
    if (key.empty()) {
      result.error = "failed to parse key at position " + std::to_string(pos);
      return result;
    }

    // Skip colon
    while (pos < json.size() &&
           std::isspace(static_cast<unsigned char>(json[pos])))
      ++pos;
    if (pos >= json.size() || json[pos] != ':') {
      result.error = "expected colon after key '" + key + "'";
      return result;
    }
    ++pos;

    // Parse value — must be a JSON string (including "").
    while (pos < json.size() &&
           std::isspace(static_cast<unsigned char>(json[pos])))
      ++pos;
    if (pos >= json.size() || json[pos] != '"') {
      result.error = "expected string value for key '" + key + "'";
      return result;
    }
    std::string value = ParseJsonString(json, pos);
    // value may be empty — that's valid JSON ""

    // Validate key and value
    if (key == "kind") {
      has_kind = true;
      if (kAllowedKinds.find(value) == kAllowedKinds.end()) {
        result.error = "invalid kind value: '" + value + "'";
        return result;
      }
      result.kind = value;
    } else if (key == "direction") {
      has_direction = true;
      if (!value.empty() &&
          kAllowedDirections.find(value) == kAllowedDirections.end()) {
        result.error = "invalid direction value: '" + value + "'";
        return result;
      }
      result.direction = value;
    } else if (key == "target") {
      has_target = true;
      if (HasControlChars(value)) {
        result.error = "target contains control characters";
        return result;
      }
      result.target = value;
    } else if (key == "reply") {
      has_reply = true;
      if (HasControlChars(value)) {
        result.error = "reply contains control characters";
        return result;
      }
      result.reply = value;
    } else {
      result.error = "unknown field: '" + key + "'";
      return result;
    }

    ++key_count;
  }

  // Step 3 — Required fields check.
  if (!has_kind) {
    result.error = "missing required field 'kind'";
    return result;
  }
  // direction, target, reply are optional (default to empty)
  (void)has_direction;
  (void)has_target;
  (void)has_reply;

  // Reply may be empty on small local models. Keep the parsed intent valid;
  // LlmIntentMapper supplies a deterministic local reply fallback.

  result.valid = true;
  return result;
}

}  // namespace k1muse_voice_intent
