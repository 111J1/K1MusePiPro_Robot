#include "k1muse_voice_intent/mock_llm_intent_client.hpp"

#include <thread>

namespace k1muse_voice_intent {

MockLlmIntentClient::MockLlmIntentClient() : name_("mock") {
  // Default JSON: query_introduce — a safe generic response.
  default_json_ = R"({"kind":"query_introduce","direction":"","target":"","reply":"我是小慕。"})";
}

bool MockLlmIntentClient::health_check(std::string* reason) {
  if (!healthy_ && reason) {
    *reason = "mock unhealthy";
  }
  return healthy_;
}

bool MockLlmIntentClient::warmup(std::string* reason) {
  // Mock client needs no warmup — always succeeds.
  (void)reason;
  return true;
}

LlmResult MockLlmIntentClient::complete_intent(
    const std::string& text,
    const LlmRequestContext& context) {
  ++call_count_;

  LlmResult result;
  auto start = std::chrono::steady_clock::now();

  // ── Force-injection flags take priority ──
  if (force_timeout_) {
    result.status = LlmStatus::kTimeout;
    result.error = "mock timeout";
    result.latency_ms = 0;
    return result;
  }
  if (force_unavailable_) {
    result.status = LlmStatus::kUnavailable;
    result.error = "mock unavailable";
    result.latency_ms = 0;
    return result;
  }

  // ── Simulated latency ──
  if (simulated_latency_ms_ > 0) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(simulated_latency_ms_));
  }

  // ── Check cancel flag ──
  if (context.cancelled && context.cancelled->load(std::memory_order_acquire)) {
    result.status = LlmStatus::kCancelled;
    result.error = "cancelled";
    auto end = std::chrono::steady_clock::now();
    result.latency_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    return result;
  }

  // ── Deadline check ──
  if (std::chrono::steady_clock::now() > context.deadline) {
    result.status = LlmStatus::kTimeout;
    result.error = "deadline exceeded";
    result.latency_ms = 0;
    return result;
  }

  // ── Unmatched text ──
  if (text.empty()) {
    result.status = LlmStatus::kInvalidResponse;
    result.error = "empty text";
    return result;
  }
  if (text.find("未知") != std::string::npos) {
    result.status = LlmStatus::kInvalidResponse;
    result.error = "unknown text";
    return result;
  }

  // ── Pattern overrides ──
  for (const auto& [pattern, json] : json_overrides_) {
    if (text.find(pattern) != std::string::npos) {
      result.status = LlmStatus::kOk;
      result.content = json;
      result.http_status = 200;
      auto end = std::chrono::steady_clock::now();
      result.latency_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
              .count();
      return result;
    }
  }

  // ── Default ──
  result.status = LlmStatus::kOk;
  result.content = default_json_;
  result.http_status = 200;
  auto end = std::chrono::steady_clock::now();
  result.latency_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  return result;
}

const std::string& MockLlmIntentClient::name() const { return name_; }

void MockLlmIntentClient::set_json_override(const std::string& pattern,
                                            const std::string& llm_json) {
  json_overrides_[pattern] = llm_json;
}

void MockLlmIntentClient::set_default_json(const std::string& llm_json) {
  default_json_ = llm_json;
}

void MockLlmIntentClient::set_healthy(bool healthy) { healthy_ = healthy; }

void MockLlmIntentClient::set_force_timeout(bool force) {
  force_timeout_ = force;
}

void MockLlmIntentClient::set_force_unavailable(bool force) {
  force_unavailable_ = force;
}

void MockLlmIntentClient::set_simulated_latency_ms(int64_t ms) {
  simulated_latency_ms_ = ms;
}

int MockLlmIntentClient::call_count() const { return call_count_; }

}  // namespace k1muse_voice_intent
