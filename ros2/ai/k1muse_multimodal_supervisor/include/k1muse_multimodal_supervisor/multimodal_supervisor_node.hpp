#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_ai_runtime_msgs/msg/ai_runtime_control.hpp"
#include "k1muse_ai_runtime_msgs/msg/ai_runtime_state.hpp"
#include "k1muse_common/msg/node_ready.hpp"
#include "k1muse_manager_msgs/msg/interaction_state.hpp"
#include "k1muse_multimodal_supervisor/interaction_state_machine.hpp"
#include "k1muse_multimodal_supervisor/runtime_control_builder.hpp"
#include "k1muse_multimodal_supervisor/target_cache.hpp"
#include "k1muse_vision_msgs/msg/detection2_d_frame.hpp"
#include "k1muse_vision_msgs/msg/target_request.hpp"
#include "k1muse_vision_msgs/msg/target_response.hpp"
#include "k1muse_voice_msgs/msg/audio_play_request.hpp"
#include "k1muse_voice_msgs/msg/audio_playback_state.hpp"
#include "k1muse_voice_msgs/msg/intent_status.hpp"
#include "k1muse_voice_msgs/msg/listen_event.hpp"
#include "k1muse_voice_msgs/msg/listen_result.hpp"
#include "k1muse_voice_msgs/msg/tts_status.hpp"
#include "k1muse_voice_msgs/msg/wakeword_event.hpp"

namespace k1muse_multimodal_supervisor {

class MultimodalSupervisorNode : public rclcpp::Node {
public:
  using AiRuntimeState = k1muse_ai_runtime_msgs::msg::AiRuntimeState;
  using AiRuntimeControl = k1muse_ai_runtime_msgs::msg::AiRuntimeControl;
  using WakewordEvent = k1muse_voice_msgs::msg::WakewordEvent;
  using ListenEvent = k1muse_voice_msgs::msg::ListenEvent;
  using ListenResult = k1muse_voice_msgs::msg::ListenResult;
  using IntentStatus = k1muse_voice_msgs::msg::IntentStatus;
  using TtsStatus = k1muse_voice_msgs::msg::TtsStatus;
  using AudioPlaybackState = k1muse_voice_msgs::msg::AudioPlaybackState;
  using AudioPlayRequest = k1muse_voice_msgs::msg::AudioPlayRequest;
  using Detection2DFrame = k1muse_vision_msgs::msg::Detection2DFrame;
  using TargetRequest = k1muse_vision_msgs::msg::TargetRequest;
  using TargetResponse = k1muse_vision_msgs::msg::TargetResponse;
  using NodeReady = k1muse_common::msg::NodeReady;
  using InteractionState = k1muse_manager_msgs::msg::InteractionState;

  explicit MultimodalSupervisorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MultimodalSupervisorNode() override;

  // Expose for testing
  State current_state() const { return state_machine_.current_state(); }

private:
  // Subscription callbacks (11 total)
  void on_runtime_state(AiRuntimeState::SharedPtr msg);
  void on_wakeword_event(WakewordEvent::SharedPtr msg);
  void on_listen_event(ListenEvent::SharedPtr msg);
  void on_listen_result(ListenResult::SharedPtr msg);
  void on_intent_status(IntentStatus::SharedPtr msg);
  void on_tts_status(TtsStatus::SharedPtr msg);
  void on_audio_playback_state(AudioPlaybackState::SharedPtr msg);
  void on_detection_2d(Detection2DFrame::SharedPtr msg);
  void on_target_request(TargetRequest::SharedPtr msg);
  void on_audio_io_state(NodeReady::SharedPtr msg);
  void on_intent_state(NodeReady::SharedPtr msg);

  // Core components (no ROS dependency)
  InteractionStateMachine state_machine_;
  RuntimeControlBuilder control_builder_;
  std::unique_ptr<TargetCache> target_cache_;

  // Mutex protecting state_machine_ and interaction state from concurrent access.
  mutable std::mutex state_mutex_;

  // Active interaction identity tracking
  std::string active_trace_id_;
  uint64_t active_epoch_ = 0;
  std::string active_request_id_;
  std::string active_utterance_id_;

  // Subsystem readiness
  bool runtime_ready_ = false;
  bool audio_ready_ = false;
  bool intent_ready_ = false;

  // No-speech timer
  rclcpp::TimerBase::SharedPtr no_speech_timer_;
  // Wakeword-cooldown timer (re-enables wakeword after no-speech timeout)
  rclcpp::TimerBase::SharedPtr wakeword_cooldown_timer_;
  // TTS dual-completion watchdog
  rclcpp::TimerBase::SharedPtr tts_completion_timer_;
  double no_speech_timeout_sec_;
  double tts_completion_timeout_sec_{15.0};

  // Publishers
  rclcpp::Publisher<AiRuntimeControl>::SharedPtr control_publisher_;
  rclcpp::Publisher<AudioPlayRequest>::SharedPtr audio_play_publisher_;
  rclcpp::Publisher<TargetResponse>::SharedPtr target_response_publisher_;
  rclcpp::Publisher<InteractionState>::SharedPtr interaction_state_publisher_;

  // Subscriptions
  rclcpp::Subscription<AiRuntimeState>::SharedPtr runtime_state_sub_;
  rclcpp::Subscription<WakewordEvent>::SharedPtr wakeword_event_sub_;
  rclcpp::Subscription<ListenEvent>::SharedPtr listen_event_sub_;
  rclcpp::Subscription<ListenResult>::SharedPtr listen_result_sub_;
  rclcpp::Subscription<IntentStatus>::SharedPtr intent_status_sub_;
  rclcpp::Subscription<TtsStatus>::SharedPtr tts_status_sub_;
  rclcpp::Subscription<AudioPlaybackState>::SharedPtr audio_playback_state_sub_;
  rclcpp::Subscription<Detection2DFrame>::SharedPtr detection_2d_sub_;
  rclcpp::Subscription<TargetRequest>::SharedPtr target_request_sub_;
  rclcpp::Subscription<NodeReady>::SharedPtr audio_io_state_sub_;
  rclcpp::Subscription<NodeReady>::SharedPtr intent_state_sub_;

  // Config
  std::string wake_ack_preset_;
  int target_cache_ttl_ms_{500};

  // Helpers
  void transition_to(State new_state, const std::string& reason);
  void publish_control();
  void publish_interaction_state(const std::string& reason);
  void start_no_speech_timer();
  void cancel_no_speech_timer();
  void start_tts_completion_timer();
  void cancel_tts_completion_timer();
  void publish_wakeword_cooldown();
  void cancel_wakeword_cooldown_timer();
  void generate_new_identity();
};

}  // namespace k1muse_multimodal_supervisor
