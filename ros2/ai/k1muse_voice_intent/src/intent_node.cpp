#include "k1muse_voice_intent/intent_node.hpp"

#include <chrono>
#include <functional>
#include <utility>

#include "k1muse_common/id_utils.hpp"
#include "k1muse_common/qos_profiles.hpp"
#include "k1muse_voice_intent/llama_server_client.hpp"
#include "k1muse_voice_intent/llm_intent_mapper.hpp"
#include "k1muse_voice_intent/mock_llm_intent_client.hpp"
#include "k1muse_voice_intent/real_llm_intent_client.hpp"

namespace k1muse_voice_intent
{

// ── Helper: build real LLM client from ROS parameters ──
// Only compiled when the real client is enabled; the node will not
// call this function when llm_backend != "real".

#ifdef K1MUSE_ENABLE_REAL_LLM_CLIENT
static std::unique_ptr<LlmIntentClient> TryCreateRealClient(
    rclcpp::Node* node) {
  LlamaServerClient::Config cfg;
  cfg.api_base = node->declare_parameter(
      "llm_api_base", std::string{"http://127.0.0.1:8080/v1"});
  cfg.model = node->declare_parameter("llm_model", std::string{"qwen2.5-0.5b"});
  cfg.max_tokens = node->declare_parameter("llm_max_tokens", cfg.max_tokens);
  cfg.temperature = node->declare_parameter("llm_temperature", cfg.temperature);
  cfg.top_p = node->declare_parameter("llm_top_p", cfg.top_p);
  cfg.top_k = node->declare_parameter("llm_top_k", cfg.top_k);
  cfg.cache_prompt = node->declare_parameter("llm_cache_prompt", true);
  cfg.use_response_format = node->has_parameter("llm_use_response_format") ?
    node->get_parameter("llm_use_response_format").as_bool() :
    node->declare_parameter("llm_use_response_format", false);
  cfg.response_format_mode = node->has_parameter("llm_response_format_mode") ?
    node->get_parameter("llm_response_format_mode").as_string() :
    node->declare_parameter("llm_response_format_mode", std::string{"none"});
  cfg.response_schema_strict = node->has_parameter("llm_response_schema_strict") ?
    node->get_parameter("llm_response_schema_strict").as_bool() :
    node->declare_parameter("llm_response_schema_strict", true);
  cfg.connect_timeout_ms = node->declare_parameter("llm_connect_timeout_ms", 500);
  cfg.request_timeout_ms = node->has_parameter("llm_request_timeout_ms") ?
    node->get_parameter("llm_request_timeout_ms").as_int() :
    node->declare_parameter("llm_request_timeout_ms", cfg.request_timeout_ms);
  cfg.low_speed_timeout_ms =
      node->declare_parameter("llm_low_speed_timeout_ms", cfg.low_speed_timeout_ms);
  cfg.max_response_bytes =
      node->declare_parameter("llm_max_response_bytes", 65536);
  cfg.system_prompt = node->declare_parameter(
      "llm_system_prompt", cfg.system_prompt);

  return std::make_unique<RealLlmIntentClient>(std::move(cfg));
}
#else
static std::unique_ptr<LlmIntentClient> TryCreateRealClient(
    rclcpp::Node* /*node*/) {
  // Real client not compiled in - caller should check llm_backend first.
  return nullptr;
}
#endif

// ── Constructors ─────────────────────────────────────────────

IntentNode::IntentNode(const rclcpp::NodeOptions & options)
: IntentNode(options, RouterConfig{}, nullptr)
{
}

IntentNode::IntentNode(const rclcpp::NodeOptions & options,
                       RouterConfig router_config,
                       std::unique_ptr<LlmIntentClient> llm_client)
: Node("intent_node", options),
  router_config_(std::move(router_config)),
  llm_client_(std::move(llm_client))
{
  // ── Router config parameters ──
  router_config_.allow_fast_intent =
    declare_parameter("allow_fast_intent", router_config_.allow_fast_intent);
  router_config_.llm_fallback_enabled =
    declare_parameter("llm_fallback_enabled", router_config_.llm_fallback_enabled);
  router_config_.complex_text_chars =
    declare_parameter("complex_text_chars", router_config_.complex_text_chars);
  router_config_.max_tts_chars =
    declare_parameter("max_tts_chars", router_config_.max_tts_chars);
  router_config_.fast_stop_max_chars =
    declare_parameter("fast_stop_max_chars", router_config_.fast_stop_max_chars);
  router_config_.fast_simple_action_max_chars =
    declare_parameter("fast_simple_action_max_chars", router_config_.fast_simple_action_max_chars);
  router_config_.fast_query_chat_max_chars =
    declare_parameter("fast_query_chat_max_chars", router_config_.fast_query_chat_max_chars);
  router_config_.fast_extractive_find_max_chars =
    declare_parameter("fast_extractive_find_max_chars", router_config_.fast_extractive_find_max_chars);
  router_config_.fast_extractive_reminder_max_chars =
    declare_parameter("fast_extractive_reminder_max_chars", router_config_.fast_extractive_reminder_max_chars);
  router_config_.fast_allow_safe_fallback_after_llm_invalid =
    declare_parameter("fast_allow_safe_fallback_after_llm_invalid",
                      router_config_.fast_allow_safe_fallback_after_llm_invalid);

  const auto busy_policy = declare_parameter("busy_policy", std::string{"reject"});
  busy_reject_ = (busy_policy != "replace");

  llm_backend_ = declare_parameter("llm_backend", std::string{"mock"});
  llm_health_interval_ms_ = declare_parameter("llm_health_interval_ms", 500);
  llm_request_timeout_ms_ = declare_parameter("llm_request_timeout_ms", 15000);
  llm_use_response_format_ = declare_parameter("llm_use_response_format", false);
  llm_response_format_mode_ = declare_parameter("llm_response_format_mode", std::string{"none"});
  llm_response_schema_strict_ = declare_parameter("llm_response_schema_strict", true);

  // ── Publishers ──
  intent_publisher_ = create_publisher<IntentLite>(
    "/voice/intent", k1muse_common::qos::ReliableResult(10));
  tts_publisher_ = create_publisher<TtsTextRequest>(
    "/voice/tts/text", k1muse_common::qos::ReliableEvent(5));
  status_publisher_ = create_publisher<IntentStatus>(
    "/voice/intent/status", k1muse_common::qos::LatchedState());
  state_publisher_ = create_publisher<NodeReady>(
    "/intent/state", k1muse_common::qos::LatchedState());

  // ── Subscription ──
  listen_result_subscription_ = create_subscription<ListenResult>(
    "/voice/listen/result", k1muse_common::qos::ReliableResult(10),
    std::bind(&IntentNode::on_listen_result, this, std::placeholders::_1));

  // ── LLM client ──
  if (!llm_client_) {
    if (llm_backend_ == "real") {
      llm_client_ = TryCreateRealClient(this);
      if (llm_client_) {
        llm_mode_ = "real";
        RCLCPP_INFO(get_logger(), "LLM backend: real (%s)",
                    llm_client_->name().c_str());
      } else {
        // Real client not compiled in - fall back to mock.
        RCLCPP_ERROR(get_logger(),
          "llm_backend=real but real client not compiled in; "
          "falling back to mock.  Rebuild with "
          "-DK1MUSE_ENABLE_REAL_LLM_CLIENT=ON");
        llm_client_ = std::make_unique<MockLlmIntentClient>();
        llm_mode_ = "mock";
      }
    } else {
      llm_client_ = std::make_unique<MockLlmIntentClient>();
      llm_mode_ = "mock";
      RCLCPP_INFO(get_logger(), "LLM backend: mock");
    }
  } else {
    llm_mode_ = llm_client_->name();
  }

  // ── Router (takes a raw pointer; node retains ownership) ──
  router_ = std::make_unique<IntentRouter>(router_config_,
                                           std::move(llm_client_));
  // llm_client_ is now null - ownership transferred to router.
  RCLCPP_INFO(get_logger(),
    "[startup] intent_node llm_backend=%s llm_mode=%s busy_policy=%s "
    "llm_fallback=%d health_interval_ms=%d request_timeout_ms=%d "
    "route_policy={stop:%d simple:%d query_chat:%d find:%d reminder:%d} "
    "response_format=%s strict=%d "
    "topics={listen_in:/voice/listen/result intent_out:/voice/intent "
    "tts_out:/voice/tts/text status:/voice/intent/status state:/intent/state}",
    llm_backend_.c_str(), llm_mode_.c_str(),
    busy_reject_ ? "reject" : "replace",
    router_config_.llm_fallback_enabled, llm_health_interval_ms_,
    llm_request_timeout_ms_, router_config_.fast_stop_max_chars,
    router_config_.fast_simple_action_max_chars,
    router_config_.fast_query_chat_max_chars,
    router_config_.fast_extractive_find_max_chars,
    router_config_.fast_extractive_reminder_max_chars,
    llm_response_format_mode_.c_str(), llm_response_schema_strict_);

  // ── Health & readiness ──
  if (llm_mode_ == "real") {
    publish_intent_state(false, llm_mode_, "llm_starting");
    health_thread_ = std::thread(&IntentNode::run_health_loop, this);
  } else {
    publish_intent_state(true, llm_mode_);
  }
}

IntentNode::~IntentNode()
{
  shutting_down_.store(true, std::memory_order_release);
  cancel_requested_.store(true, std::memory_order_release);
  // Interrupt in-flight libcurl call before joining worker.
  if (active_cancel_flag_) {
    active_cancel_flag_->store(true, std::memory_order_release);
  }

  if (health_thread_.joinable()) {
    health_thread_.join();
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

// ── Accessors ────────────────────────────────────────────────

uint64_t IntentNode::latest_epoch() const
{
  return latest_epoch_.load(std::memory_order_acquire);
}

const RouterConfig & IntentNode::router_config() const
{
  return router_config_;
}

LlmIntentClient* IntentNode::llm_client() const {
  return router_ ? router_->client() : nullptr;
}

// ── Health loop (real backend only) ─────────────────────────

void IntentNode::run_health_loop() {
  RCLCPP_INFO(get_logger(), "Health loop started");

  while (!shutting_down_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(llm_health_interval_ms_));

    if (shutting_down_.load(std::memory_order_acquire)) break;

    auto* client = router_->client();
    if (!client) {
      publish_intent_state(false, llm_mode_, "no client");
      break;
    }

    std::string reason;
    bool ok = client->health_check(&reason);

    if (ok && !llm_ready_.load(std::memory_order_acquire)) {
      // ── Warmup: prime llama-server KV cache with system prompt ──
      RCLCPP_INFO(get_logger(), "LLM health OK, running warmup...");
      std::string warmup_reason;
      if (client->warmup(&warmup_reason)) {
        RCLCPP_INFO(get_logger(), "LLM warmup OK, publishing ready");
        publish_intent_state(true, llm_mode_, "health ok");
      } else {
    RCLCPP_WARN(get_logger(), "LLM warmup failed: %s", warmup_reason.c_str());
        publish_intent_state(true, llm_mode_,
          "health ok (warmup failed: " + warmup_reason + ")");
      }
    } else if (!ok) {
      RCLCPP_WARN(get_logger(), "LLM health check failed: %s", reason.c_str());
      if (llm_ready_.load(std::memory_order_acquire)) {
        publish_intent_state(false, llm_mode_, reason);
      }
    }
  }
}

// ── publish_intent_state ─────────────────────────────────────

void IntentNode::publish_intent_state(bool ready, const std::string & mode,
                                      const std::string & reason) {
  llm_ready_.store(ready, std::memory_order_release);
  NodeReady msg;
  msg.header.stamp = now();
  msg.node_name = "intent_node";
  msg.ready = ready;
  msg.mode = mode;
  msg.reason = reason;
  state_publisher_->publish(msg);
}

// ── on_listen_result ─────────────────────────────────────────

void IntentNode::on_listen_result(ListenResult::SharedPtr msg)
{
  if (shutting_down_.load(std::memory_order_acquire)) {
    return;
  }

  if (!msg->success || msg->text.empty()) {
    RCLCPP_WARN(get_logger(),
      "[trace] listen_result rejected trace_id=%s utterance_id=%s epoch=%llu "
      "success=%d text_len=%zu reason=%s",
      msg->trace_id.c_str(), msg->utterance_id.c_str(),
      static_cast<unsigned long long>(msg->epoch), msg->success,
      msg->text.size(), msg->reason.c_str());
    IntentStatus status_msg;
    status_msg.header.stamp = now();
    status_msg.trace_id = msg->trace_id;
    status_msg.request_id = k1muse_common::make_id("req");
    status_msg.utterance_id = msg->utterance_id;
    status_msg.epoch = msg->epoch;
    status_msg.state = IntentStatus::STATE_FAILED;
    status_msg.state_name = "FAILED";
    status_msg.reason = msg->success ? "empty text" : msg->reason;
    status_msg.has_tts = false;
    status_publisher_->publish(status_msg);
    return;
  }

  if (busy_.load(std::memory_order_acquire)) {
    if (busy_reject_) {
      RCLCPP_WARN(get_logger(),
        "[trace] intent request rejected trace_id=%s utterance_id=%s epoch=%llu reason=busy",
        msg->trace_id.c_str(), msg->utterance_id.c_str(),
        static_cast<unsigned long long>(msg->epoch));
      IntentStatus status_msg;
      status_msg.header.stamp = now();
      status_msg.trace_id = msg->trace_id;
      status_msg.request_id = k1muse_common::make_id("req");
      status_msg.utterance_id = msg->utterance_id;
      status_msg.epoch = msg->epoch;
      status_msg.state = IntentStatus::STATE_FAILED;
      status_msg.state_name = "FAILED";
      status_msg.reason = "busy";
      status_msg.has_tts = false;
      status_publisher_->publish(status_msg);
      return;
    }
    RCLCPP_INFO(get_logger(),
      "[trace] intent request replacing trace_id=%s utterance_id=%s epoch=%llu",
      msg->trace_id.c_str(), msg->utterance_id.c_str(),
      static_cast<unsigned long long>(msg->epoch));
  }

  latest_epoch_.store(msg->epoch, std::memory_order_release);
  // Signal the old worker via the shared cancel flag so in-flight
  // libcurl calls are interrupted before we join.
  if (active_cancel_flag_) {
    active_cancel_flag_->store(true, std::memory_order_release);
  }
  cancel_requested_.store(true);
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  cancel_requested_.store(false);
  // Fresh cancel flag for the new request.
  active_cancel_flag_ = std::make_shared<std::atomic_bool>(false);
  busy_.store(true, std::memory_order_release);
  worker_thread_ = std::thread(&IntentNode::process_intent, this, *msg);
}

// ── process_intent ───────────────────────────────────────────

void IntentNode::process_intent(ListenResult msg)
{
  const uint64_t epoch = msg.epoch;
  const std::string trace_id = msg.trace_id;
  const std::string utterance_id = msg.utterance_id;
  const std::string request_id = k1muse_common::make_id("req");
  const std::string text = msg.text;
  const auto request_start = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(),
    "[trace] intent request start trace_id=%s request_id=%s utterance_id=%s "
    "epoch=%llu text_len=%zu llm_mode=%s",
    trace_id.c_str(), request_id.c_str(), utterance_id.c_str(),
    static_cast<unsigned long long>(epoch), text.size(), llm_mode_.c_str());

  auto publish_cancelled = [&]() {
    IntentStatus status_msg;
    status_msg.header.stamp = now();
    status_msg.trace_id = trace_id;
    status_msg.request_id = request_id;
    status_msg.utterance_id = utterance_id;
    status_msg.epoch = epoch;
    status_msg.state = IntentStatus::STATE_FAILED;
    status_msg.state_name = "FAILED";
    status_msg.reason = "cancelled";
    status_msg.has_tts = false;
    status_publisher_->publish(status_msg);
  };

  if (cancel_requested_.load(std::memory_order_acquire)) {
    publish_cancelled();
    busy_.store(false, std::memory_order_release);
    return;
  }

  try {
    // ── Publish ACTIVE status ──
    {
      IntentStatus status_msg;
      status_msg.header.stamp = now();
      status_msg.trace_id = trace_id;
      status_msg.request_id = request_id;
      status_msg.utterance_id = utterance_id;
      status_msg.epoch = epoch;
      status_msg.state = IntentStatus::STATE_ACTIVE;
      status_msg.state_name = "ACTIVE";
      status_publisher_->publish(status_msg);
    }

    // ── Build LlmRequestContext for cancel + deadline ──
    // Uses the shared cancel flag so on_listen_result can interrupt
    // in-flight libcurl calls before joining the worker.
    auto cancelled_flag = active_cancel_flag_;
    if (!cancelled_flag) {
      cancelled_flag = std::make_shared<std::atomic_bool>(false);
    }
    LlmRequestContext llm_ctx;
    llm_ctx.request_id = request_id;
    llm_ctx.epoch = epoch;
    llm_ctx.deadline = std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(llm_request_timeout_ms_);
    llm_ctx.cancelled = cancelled_flag;

    // ── Process through router ──
    RouterResult result = router_->process(text, &llm_ctx);
    const auto request_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - request_start).count();

    // ── Stale epoch check ──
    if (epoch != latest_epoch_.load(std::memory_order_acquire)) {
      RCLCPP_INFO(get_logger(), "Stale epoch %lu (latest=%lu), dropping result",
                  epoch, latest_epoch_.load(std::memory_order_acquire));
      publish_cancelled();
      busy_.store(false, std::memory_order_release);
      return;
    }

    if (cancel_requested_.load(std::memory_order_acquire)) {
      publish_cancelled();
      busy_.store(false, std::memory_order_release);
      return;
    }

    // ── Publish IntentLite ──
    IntentLite intent_msg;
    intent_msg.header.stamp = now();
    intent_msg.trace_id = trace_id;
    intent_msg.request_id = request_id;
    intent_msg.utterance_id = utterance_id;
    intent_msg.epoch = epoch;

    bool tts_sent = false;

    if (!result.failed) {
      IntentLiteFields fields;
      IntentRouter::to_intent_lite(result, trace_id, request_id,
                                   utterance_id, epoch, fields);
      intent_msg.intent_name = fields.intent_name;
      intent_msg.target = fields.target;
      intent_msg.action = fields.action;
      intent_msg.location = fields.location;
      intent_msg.value = fields.value;
      intent_msg.confidence = fields.confidence;
      intent_msg.requires_confirmation = fields.requires_confirmation;
      intent_msg.distance_m = fields.distance_m;
      intent_msg.angle_rad = fields.angle_rad;
      intent_publisher_->publish(intent_msg);

      RCLCPP_INFO(get_logger(),
        "[trace] intent result trace_id=%s request_id=%s utterance_id=%s "
        "epoch=%llu route_source=%s fast_category=%s fast_slot=%s "
        "fast_reject_reason=%s llm_status=%s response_format=%s "
        "name=%s action=%s target=%s value=%s conf=%.2f "
        "tts_len=%zu request_ms=%lld",
        trace_id.c_str(), request_id.c_str(), utterance_id.c_str(),
        static_cast<unsigned long long>(epoch), result.route_source.c_str(),
        result.fast_category.c_str(), result.fast_slot_state.c_str(),
        result.fast_reject_reason.c_str(), result.llm_status.c_str(),
        llm_response_format_mode_.c_str(), fields.intent_name.c_str(),
        fields.action.c_str(), fields.target.c_str(), fields.value.c_str(),
        fields.confidence, result.decision.tts_reply.size(),
        static_cast<long long>(request_ms));

      if (!result.failure_reason.empty()) {
        RCLCPP_WARN(get_logger(),
          "[trace] intent recovered trace_id=%s request_id=%s utterance_id=%s "
          "epoch=%llu reason=%s route_source=%s fast_reject_reason=%s "
          "llm_status=%s name=%s action=%s tts_len=%zu request_ms=%lld",
          trace_id.c_str(), request_id.c_str(), utterance_id.c_str(),
          static_cast<unsigned long long>(epoch), result.failure_reason.c_str(),
          result.route_source.c_str(), result.fast_reject_reason.c_str(),
          result.llm_status.c_str(), fields.intent_name.c_str(),
          fields.action.c_str(), result.decision.tts_reply.size(),
          static_cast<long long>(request_ms));
      }

      // ── TTS reply ──
      TtsRequestFields tts_fields;
      if (IntentRouter::to_tts_request(result, trace_id, request_id,
                                       epoch, tts_fields)) {
        TtsTextRequest tts_msg;
        tts_msg.header.stamp = now();
        tts_msg.trace_id = trace_id;
        tts_msg.request_id = request_id;
        tts_msg.epoch = epoch;
        tts_msg.source = tts_fields.source;
        tts_msg.priority = tts_fields.priority;
        tts_msg.text = tts_fields.text;
        tts_msg.voice = tts_fields.voice;
        tts_publisher_->publish(tts_msg);
        tts_sent = true;
      }
    }

    // ── Publish FINISHED / FAILED ──
    {
      IntentStatus status_msg;
      status_msg.header.stamp = now();
      status_msg.trace_id = trace_id;
      status_msg.request_id = request_id;
      status_msg.utterance_id = utterance_id;
      status_msg.epoch = epoch;
      if (result.failed) {
        RCLCPP_WARN(get_logger(),
          "[trace] intent failed trace_id=%s request_id=%s utterance_id=%s "
          "epoch=%llu reason=%s route_source=%s fast_reject_reason=%s "
          "llm_status=%s request_ms=%lld",
          trace_id.c_str(), request_id.c_str(), utterance_id.c_str(),
          static_cast<unsigned long long>(epoch), result.failure_reason.c_str(),
          result.route_source.c_str(), result.fast_reject_reason.c_str(),
          result.llm_status.c_str(), static_cast<long long>(request_ms));
        status_msg.state = IntentStatus::STATE_FAILED;
        status_msg.state_name = "FAILED";
        status_msg.reason = result.failure_reason;
        status_msg.has_tts = false;
      } else {
        status_msg.state = IntentStatus::STATE_FINISHED;
        status_msg.state_name = "FINISHED";
        status_msg.reason = result.failure_reason;
        status_msg.has_tts = tts_sent;
      }
      status_publisher_->publish(status_msg);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Intent processing failed: %s", e.what());
    IntentStatus status_msg;
    status_msg.header.stamp = now();
    status_msg.trace_id = trace_id;
    status_msg.request_id = request_id;
    status_msg.utterance_id = utterance_id;
    status_msg.epoch = epoch;
    status_msg.state = IntentStatus::STATE_FAILED;
    status_msg.state_name = "FAILED";
    status_msg.reason = e.what();
    status_msg.has_tts = false;
    status_publisher_->publish(status_msg);
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Intent processing failed with non-standard exception");
    IntentStatus status_msg;
    status_msg.header.stamp = now();
    status_msg.trace_id = trace_id;
    status_msg.request_id = request_id;
    status_msg.utterance_id = utterance_id;
    status_msg.epoch = epoch;
    status_msg.state = IntentStatus::STATE_FAILED;
    status_msg.state_name = "FAILED";
    status_msg.reason = "internal error";
    status_msg.has_tts = false;
    status_publisher_->publish(status_msg);
  }

  busy_.store(false, std::memory_order_release);
}

}  // namespace k1muse_voice_intent
