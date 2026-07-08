#include "k1muse_voice_intent/llama_server_client.hpp"

#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

#ifdef K1MUSE_ENABLE_REAL_LLM_CLIENT

#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace k1muse_voice_intent {

// ═════════════════════════════════════════════════════════════════
// curl_global_init — once per process (matches SpacemiT SDK pattern)
// ═════════════════════════════════════════════════════════════════
static std::once_flag g_curl_init_flag;

static void EnsureCurlInit() {
  std::call_once(g_curl_init_flag, []() {
    curl_global_init(CURL_GLOBAL_DEFAULT);
  });
}

// ═════════════════════════════════════════════════════════════════
// Write callback — accumulates response body
// ═════════════════════════════════════════════════════════════════
struct WriteContext {
  std::string buffer;
  size_t max_bytes = 65536;
  bool truncated = false;
};

static size_t WriteCallback(void* contents, size_t size, size_t nmemb,
                            void* userp) {
  auto* ctx = static_cast<WriteContext*>(userp);
  size_t total = size * nmemb;
  if (ctx->buffer.size() + total > ctx->max_bytes) {
    ctx->truncated = true;
    return 0;  // Signal error to curl
  }
  ctx->buffer.append(static_cast<const char*>(contents), total);
  return total;
}

// ═════════════════════════════════════════════════════════════════
// Progress callback — supports cancellation and deadline
// ═════════════════════════════════════════════════════════════════
struct ProgressContext {
  const LlmRequestContext* request_ctx = nullptr;
};

static int ProgressCallback(void* clientp, curl_off_t /*dltotal*/,
                            curl_off_t /*dlnow*/, curl_off_t /*ultotal*/,
                            curl_off_t /*ulnow*/) {
  auto* ctx = static_cast<ProgressContext*>(clientp);
  if (!ctx || !ctx->request_ctx) return 0;

  // Check cancel flag
  if (ctx->request_ctx->cancelled &&
      ctx->request_ctx->cancelled->load(std::memory_order_acquire)) {
    return 1;  // Non-zero → abort transfer
  }

  // Check deadline
  if (std::chrono::steady_clock::now() > ctx->request_ctx->deadline) {
    return 1;
  }

  return 0;  // Continue
}

// ═════════════════════════════════════════════════════════════════
// Helper: set common curl options
// ═════════════════════════════════════════════════════════════════
static void SetupCommonOpts(CURL* curl, const LlamaServerClient::Config& cfg,
                            struct curl_slist* headers) {
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS,
                   static_cast<long>(cfg.connect_timeout_ms));
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS,
                   static_cast<long>(cfg.request_timeout_ms));
  curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
  curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME,
                   static_cast<long>(cfg.low_speed_timeout_ms / 1000));
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
}

// ═════════════════════════════════════════════════════════════════
// PIMPL
// ═════════════════════════════════════════════════════════════════
struct LlamaServerClientImpl {
  LlamaServerClient::Config config;
};

LlamaServerClient::LlamaServerClient(Config config)
    : impl_(std::make_unique<LlamaServerClientImpl>()) {
  impl_->config = std::move(config);
  EnsureCurlInit();
}

LlamaServerClient::~LlamaServerClient() = default;

// ── health_check ──────────────────────────────────────────────

bool LlamaServerClient::health_check(std::string* reason) {
  CURL* curl = curl_easy_init();
  if (!curl) {
    if (reason) *reason = "curl_easy_init failed";
    return false;
  }

  std::string url = impl_->config.api_base;
  // Strip trailing "/v1" if present, then append "/health"
  if (url.size() >= 3 && url.substr(url.size() - 3) == "/v1") {
    url = url.substr(0, url.size() - 3);
  }
  url += "/health";

  WriteContext write_ctx;
  write_ctx.max_bytes = 1024;

  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 500L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 3000L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &write_ctx);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;  // NOLINT(runtime/int)
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    if (reason) *reason = std::string("curl error: ") + curl_easy_strerror(res);
    return false;
  }

  if (http_code == 200) {
    // Parse JSON to confirm status == "ok"
    try {
      auto j = nlohmann::json::parse(write_ctx.buffer);
      if (j.contains("status") && j["status"] == "ok") {
        return true;
      }
      if (reason) *reason = "status not ok: " + write_ctx.buffer;
      return false;
    } catch (...) {
      if (reason) *reason = "health response parse error";
      return false;
    }
  }

  if (http_code == 503) {
    // Model still loading
    if (reason) {
      try {
        auto j = nlohmann::json::parse(write_ctx.buffer);
        if (j.contains("error") && j["error"].contains("message")) {
          *reason = j["error"]["message"].get<std::string>();
        } else {
          *reason = "model loading";
        }
      } catch (...) {
        *reason = "model loading (503)";
      }
    }
    return false;
  }

  if (reason) *reason = "HTTP " + std::to_string(http_code);
  return false;
}

static nlohmann::json BuildIntentJsonSchema(bool strict) {
  return {
    {"name", "k1muse_intent"},
    {"strict", strict},
    {"schema", {
      {"type", "object"},
      {"additionalProperties", false},
      {"required", {"kind", "direction", "target", "reply"}},
      {"properties", {
        {"kind", {
          {"type", "string"},
          {"enum", {
            "move", "stop", "rotate", "lift", "find",
            "query_introduce", "query_time",
            "reminder_create", "reminder_list",
            "system_shutdown", "system_reboot",
            "chat", "ask_repeat", "unknown"}}}},
        {"direction", {
          {"type", "string"},
          {"enum", {"", "forward", "backward", "left", "right", "up", "down"}}}},
        {"target", {{"type", "string"}}},
        {"reply", {{"type", "string"}}},
      }},
    }},
  };
}

static void MaybeAddResponseFormat(nlohmann::json* request,
                                   const LlamaServerClient::Config& cfg) {
  if (!cfg.use_response_format || cfg.response_format_mode == "none") {
    return;
  }

  if (cfg.response_format_mode == "json_schema") {
    (*request)["response_format"] = {
      {"type", "json_schema"},
      {"json_schema", BuildIntentJsonSchema(cfg.response_schema_strict)},
    };
  } else if (cfg.response_format_mode == "json_object") {
    // Do not attach a schema to json_object mode. Board builds may only treat
    // this as a coarse JSON-object constraint; LlmResponseValidator remains
    // the strict semantic contract.
    (*request)["response_format"] = {{"type", "json_object"}};
  }
}
// ── complete ──────────────────────────────────────────────────

LlmResult LlamaServerClient::complete(const std::string& user_text,
                                      const LlmRequestContext& context) {
  LlmResult result;
  auto start = std::chrono::steady_clock::now();

  CURL* curl = curl_easy_init();
  if (!curl) {
    result.status = LlmStatus::kInternalError;
    result.error = "curl_easy_init failed";
    return result;
  }

  // ── Build request JSON ──
  nlohmann::json request;
  request["model"] = impl_->config.model;
  request["max_tokens"] = impl_->config.max_tokens;
  request["temperature"] = impl_->config.temperature;
  request["top_p"] = impl_->config.top_p;
  request["top_k"] = impl_->config.top_k;
  request["cache_prompt"] = impl_->config.cache_prompt;
  request["stream"] = false;  // Non-streaming for simple intent
  MaybeAddResponseFormat(&request, impl_->config);

  nlohmann::json messages = nlohmann::json::array();
  if (!impl_->config.system_prompt.empty()) {
    messages.push_back(
        {{"role", "system"}, {"content", impl_->config.system_prompt}});
  }
  messages.push_back({{"role", "user"}, {"content", user_text}});
  request["messages"] = messages;

  std::string json_data = request.dump();

  // ── Headers ──
  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  // llama-server default auth requirement: try without first.
  // If server returns 401, we add the no-key header.  For simplicity
  // we include it always — llama-server ignores unknown auth headers.
  headers = curl_slist_append(headers, "Authorization: Bearer no-key");

  // ── Set up curl ──
  std::string url = impl_->config.api_base + "/chat/completions";

  WriteContext write_ctx;
  write_ctx.max_bytes =
      static_cast<size_t>(impl_->config.max_response_bytes);

  ProgressContext progress_ctx;
  progress_ctx.request_ctx = &context;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
  SetupCommonOpts(curl, impl_->config, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &write_ctx);

  // Progress callback for cancel/deadline
  curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, ProgressCallback);
  curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &progress_ctx);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;  // NOLINT(runtime/int)
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  auto end = std::chrono::steady_clock::now();
  result.latency_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  result.http_status = static_cast<int>(http_code);

  // ── Classify result ──

  // Cancelled via progress callback
  if (context.cancelled &&
      context.cancelled->load(std::memory_order_acquire)) {
    result.status = LlmStatus::kCancelled;
    result.error = "cancelled";
    return result;
  }

  // Deadline exceeded
  if (end > context.deadline) {
    result.status = LlmStatus::kTimeout;
    result.error = "deadline exceeded";
    return result;
  }

  // Response too large
  if (write_ctx.truncated) {
    result.status = LlmStatus::kResponseTooLarge;
    result.error = "response exceeded " +
                   std::to_string(impl_->config.max_response_bytes) + " bytes";
    return result;
  }

  // Curl error
  if (res != CURLE_OK) {
    if (res == CURLE_OPERATION_TIMEDOUT) {
      result.status = LlmStatus::kTimeout;
      result.error = "curl timeout";
    } else if (res == CURLE_ABORTED_BY_CALLBACK) {
      result.status = LlmStatus::kCancelled;
      result.error = "cancelled via callback";
    } else {
      result.status = LlmStatus::kUnavailable;
      result.error =
          std::string("curl error: ") + curl_easy_strerror(res);
    }
    return result;
  }

  // HTTP error
  if (http_code != 200) {
    result.status = LlmStatus::kHttpError;
    result.error = "HTTP " + std::to_string(http_code);
    return result;
  }

  // ── Extract content from response ──
  try {
    auto j = nlohmann::json::parse(write_ctx.buffer);

    // OpenAI-compatible: choices[0].message.content
    if (j.contains("choices") && j["choices"].is_array() &&
        !j["choices"].empty()) {
      auto& choice = j["choices"][0];
      if (choice.contains("message") && choice["message"].is_object() &&
          choice["message"].contains("content") &&
          !choice["message"]["content"].is_null()) {
        result.content = choice["message"]["content"].get<std::string>();
        result.status = LlmStatus::kOk;
        return result;
      }
    }

    result.status = LlmStatus::kInvalidResponse;
    result.error = "response missing choices[0].message.content";
    return result;
  } catch (const nlohmann::json::exception& e) {
    result.status = LlmStatus::kInvalidResponse;
    result.error = std::string("JSON parse error: ") + e.what();
    return result;
  }
}

// ── warmup ────────────────────────────────────────────────────

bool LlamaServerClient::warmup(std::string* reason) {
  CURL* curl = curl_easy_init();
  if (!curl) {
    if (reason) *reason = "curl_easy_init failed";
    return false;
  }

  // Minimal request: system prompt + one short user message.
  // cache_prompt=true causes the server to compute and cache the KV
  // state for the system prompt, so subsequent complete() calls skip prefill.
  nlohmann::json request;
  request["model"] = impl_->config.model;
  request["max_tokens"] = 1;
  request["temperature"] = 0.0;
  request["cache_prompt"] = true;
  request["stream"] = false;

  nlohmann::json messages = nlohmann::json::array();
  if (!impl_->config.system_prompt.empty()) {
    messages.push_back(
        {{"role", "system"}, {"content", impl_->config.system_prompt}});
  }
  messages.push_back({{"role", "user"}, {"content", "hello"}});
  request["messages"] = messages;

  std::string json_data = request.dump();

  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, "Authorization: Bearer no-key");

  std::string url = impl_->config.api_base + "/chat/completions";

  WriteContext write_ctx;
  write_ctx.max_bytes = 1024;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &write_ctx);
  // No overall timeout for warmup — model loading may take minutes.
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 0L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS,
                   static_cast<long>(impl_->config.connect_timeout_ms));
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;  // NOLINT(runtime/int)
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    if (reason) *reason = std::string("warmup curl error: ") + curl_easy_strerror(res);
    return false;
  }
  if (http_code != 200) {
    if (reason) *reason = "warmup HTTP " + std::to_string(http_code);
    return false;
  }
  return true;
}

}  // namespace k1muse_voice_intent

#else  // !K1MUSE_ENABLE_REAL_LLM_CLIENT

// ── Stub implementation when real LLM is not compiled in ──

namespace k1muse_voice_intent {

struct LlamaServerClientImpl {
  LlamaServerClient::Config config;
};

LlamaServerClient::LlamaServerClient(Config config)
    : impl_(std::make_unique<LlamaServerClientImpl>()) {
  impl_->config = std::move(config);
}

LlamaServerClient::~LlamaServerClient() = default;

bool LlamaServerClient::health_check(std::string* reason) {
  if (reason) *reason = "real LLM client not compiled in";
  return false;
}

LlmResult LlamaServerClient::complete(const std::string& /*user_text*/,
                                      const LlmRequestContext& /*context*/) {
  LlmResult result;
  result.status = LlmStatus::kUnavailable;
  result.error = "real LLM client not compiled in";
  return result;
}

bool LlamaServerClient::warmup(std::string* reason) {
  if (reason) *reason = "real LLM client not compiled in";
  return false;
}

}  // namespace k1muse_voice_intent

#endif  // K1MUSE_ENABLE_REAL_LLM_CLIENT
