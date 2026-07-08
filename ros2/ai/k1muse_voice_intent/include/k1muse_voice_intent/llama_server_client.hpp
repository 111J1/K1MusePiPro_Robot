#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "k1muse_voice_intent/intent_types.hpp"

namespace k1muse_voice_intent {

// Opaque implementation (PIMPL) — libcurl details stay in .cpp.
struct LlamaServerClientImpl;

// Lightweight HTTP client for llama-server's OpenAI-compatible API.
//
// Responsibilities:
//  - GET /health  →  check if the model is loaded and ready
//  - POST /v1/chat/completions  →  send user text, receive JSON
//
// Thread-safety: each instance should be used from one thread.
// The progress callback supports cancellation via LlmRequestContext.
//
// This class is compiled only when K1MUSE_ENABLE_REAL_LLM_CLIENT=ON.
class LlamaServerClient {
 public:
  struct Config {
    std::string api_base = "http://127.0.0.1:8080/v1";
    std::string model = "qwen2.5-0.5b";
    std::string system_prompt =
        "你是 K1 机器人意图分类器。只输出一行 JSON，不输出解释。\n"
        "格式固定：{\"kind\":\"\",\"direction\":\"\",\"target\":\"\",\"reply\":\"\"}\n"
        "kind 只能是：move, stop, rotate, lift, find, query_introduce, "
        "query_time, reminder_create, reminder_list, system_shutdown, "
        "system_reboot, chat, ask_repeat, unknown。\n"
        "规则：前进/后退/左转/右转=>move；停下/别动/不要动=>stop；"
        "升高/降低=>lift；找/搜索/帮我拿/拿过来=>find；"
        "你是谁/你能做什么/你会什么=>query_introduce；"
        "你好/谢谢/再见/辛苦了=>chat；"
        "提醒我/记得叫我/设置提醒=>reminder_create，target 填完整提醒内容，内容为空则 ask_repeat；"
        "查看提醒/提醒列表=>reminder_list；不确定或无法执行=>unknown；reply 必须是简短中文，不能空。\n"
        "用户：你能做什么\n"
        "{\"kind\":\"query_introduce\",\"direction\":\"\",\"target\":\"\",\"reply\":\"我可以移动、找东西、回答问题和设置提醒。\"}\n"
        "用户：帮我把杯子拿过来\n"
        "{\"kind\":\"find\",\"direction\":\"\",\"target\":\"杯子\",\"reply\":\"好的，我先帮你找杯子。\"}\n"
        "用户：不要动\n"
        "{\"kind\":\"stop\",\"direction\":\"\",\"target\":\"\",\"reply\":\"已停止。\"}\n"
        "用户：三分钟后提醒我关火\n"
        "{\"kind\":\"reminder_create\",\"direction\":\"\",\"target\":\"三分钟后关火\",\"reply\":\"好的，我会提醒你。\"}";
    int max_tokens = 96;
    double temperature = 0.0;
    double top_p = 0.3;
    int top_k = 1;
    bool cache_prompt = true;
    bool use_response_format = false;
    std::string response_format_mode = "none";  // none | json_schema | json_object
    bool response_schema_strict = true;
    int connect_timeout_ms = 500;
    int request_timeout_ms = 15000;
    int low_speed_timeout_ms = 15000;
    int max_response_bytes = 65536;
  };

  explicit LlamaServerClient(Config config);
  ~LlamaServerClient();

  // Non-copyable (libcurl handles are not trivially copyable).
  LlamaServerClient(const LlamaServerClient&) = delete;
  LlamaServerClient& operator=(const LlamaServerClient&) = delete;

  // Health check.  Returns true when the model is loaded and ready.
  // Sets `reason` when returning false (e.g. "loading", "connection refused").
  bool health_check(std::string* reason = nullptr);

  // Send a chat completion request.
  //
  // Blocks until the response is received, the deadline expires, or
  // cancellation is requested.  Observes context.cancelled and
  // context.deadline via libcurl progress callback.
  //
  // Returns LlmResult with:
  //   - kOk + content = raw JSON string from the model
  //   - kTimeout / kCancelled / kHttpError / …
  LlmResult complete(const std::string& user_text,
                     const LlmRequestContext& context);

  // Prime the KV cache with the system prompt.
  // Sends system_prompt + "hello" with max_tokens=1 and cache_prompt=true,
  // so subsequent complete() calls skip system-prompt prefill.
  // Returns true on HTTP 2xx.  Sets `reason` when returning false.
  bool warmup(std::string* reason = nullptr);

 private:
  std::unique_ptr<LlamaServerClientImpl> impl_;
};

}  // namespace k1muse_voice_intent
