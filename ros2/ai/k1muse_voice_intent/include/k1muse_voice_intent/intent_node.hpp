#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_common/msg/node_ready.hpp"
#include "k1muse_voice_intent/intent_router.hpp"
#include "k1muse_voice_msgs/msg/intent_lite.hpp"
#include "k1muse_voice_msgs/msg/intent_status.hpp"
#include "k1muse_voice_msgs/msg/listen_result.hpp"
#include "k1muse_voice_msgs/msg/tts_text_request.hpp"

namespace k1muse_voice_intent
{

class IntentNode : public rclcpp::Node
{
public:
  using ListenResult = k1muse_voice_msgs::msg::ListenResult;
  using IntentLite = k1muse_voice_msgs::msg::IntentLite;
  using IntentStatus = k1muse_voice_msgs::msg::IntentStatus;
  using TtsTextRequest = k1muse_voice_msgs::msg::TtsTextRequest;
  using NodeReady = k1muse_common::msg::NodeReady;

  explicit IntentNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~IntentNode();

  // Test constructor: accepts a pre-built RouterConfig and pre-built
  // LlmIntentClient (the node takes ownership).
  IntentNode(const rclcpp::NodeOptions & options,
             RouterConfig router_config,
             std::unique_ptr<LlmIntentClient> llm_client);

  // Public for testing.
  void on_listen_result(ListenResult::SharedPtr msg);
  void publish_intent_state(bool ready, const std::string & mode,
                            const std::string & reason = {});

  // Test accessors.
  uint64_t latest_epoch() const;
  const RouterConfig & router_config() const;
  LlmIntentClient* llm_client() const;

private:
  void process_intent(ListenResult msg);
  void run_health_loop();

  // Publishers
  rclcpp::Publisher<IntentLite>::SharedPtr intent_publisher_;
  rclcpp::Publisher<TtsTextRequest>::SharedPtr tts_publisher_;
  rclcpp::Publisher<IntentStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<NodeReady>::SharedPtr state_publisher_;

  // Subscription
  rclcpp::Subscription<ListenResult>::SharedPtr listen_result_subscription_;

  // Router
  std::unique_ptr<IntentRouter> router_;
  RouterConfig router_config_;

  // LLM client - owned separately so we can access health_check().
  std::unique_ptr<LlmIntentClient> llm_client_;

  // Worker
  std::atomic<bool> busy_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> shutting_down_{false};
  std::thread worker_thread_;
  // Shared cancel flag wired into LlmRequestContext so replace-mode
  // cancels can interrupt in-flight libcurl calls.
  std::shared_ptr<std::atomic_bool> active_cancel_flag_;

  // Health
  std::thread health_thread_;
  std::atomic<bool> llm_ready_{false};
  std::string llm_mode_;  // "mock" or "real"

  // Config
  bool busy_reject_;  // true=reject, false=replace

  // llm_backend parameter: "mock" (default) or "real"
  std::string llm_backend_;

  // Health polling interval (ms)
  int llm_health_interval_ms_ = 500;

  // Per-request LLM deadline (ms).
  int llm_request_timeout_ms_ = 60000;

  bool llm_use_response_format_ = false;
  std::string llm_response_format_mode_ = "none";
  bool llm_response_schema_strict_ = true;

  // Epoch tracking
  std::atomic<uint64_t> latest_epoch_{0};
};

}  // namespace k1muse_voice_intent
