#include "k1muse_multimodal_supervisor/multimodal_supervisor_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <functional>
#include <mutex>
#include <utility>

#include "k1muse_common/id_utils.hpp"
#include "k1muse_common/qos_profiles.hpp"

namespace k1muse_multimodal_supervisor {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MultimodalSupervisorNode::MultimodalSupervisorNode(
    const rclcpp::NodeOptions& options)
    : Node("multimodal_supervisor", options) {
  // Parameters declared in body to avoid RISC-V initializer-list issues.
  target_cache_ttl_ms_ =
      static_cast<int>(declare_parameter("target_cache_ttl_ms", 500));
  no_speech_timeout_sec_ = declare_parameter("no_speech_timeout_sec", 10.0);
  tts_completion_timeout_sec_ = declare_parameter("tts_completion_timeout_sec", 15.0);
  wake_ack_preset_ = declare_parameter("wake_ack_preset", std::string{"wake_ack"});
  target_cache_ = std::make_unique<TargetCache>(std::chrono::milliseconds(target_cache_ttl_ms_));

  // Publishers
  control_publisher_ = create_publisher<AiRuntimeControl>(
      "/ai_runtime/control", k1muse_common::qos::LatchedState());
  audio_play_publisher_ = create_publisher<AudioPlayRequest>(
      "/voice/audio/play", k1muse_common::qos::ReliableEvent(5));
  target_response_publisher_ = create_publisher<TargetResponse>(
      "/vision/target_response", k1muse_common::qos::ReliableEvent(5));
  interaction_state_publisher_ = create_publisher<InteractionState>(
      "/robot/interaction_state", k1muse_common::qos::LatchedState());

  // Subscriptions
  runtime_state_sub_ = create_subscription<AiRuntimeState>(
      "/ai_runtime/state", k1muse_common::qos::LatchedState(),
      std::bind(&MultimodalSupervisorNode::on_runtime_state, this, std::placeholders::_1));

  wakeword_event_sub_ = create_subscription<WakewordEvent>(
      "/ai_runtime/wakeword/event", k1muse_common::qos::ReliableEvent(5),
      std::bind(&MultimodalSupervisorNode::on_wakeword_event, this, std::placeholders::_1));

  listen_event_sub_ = create_subscription<ListenEvent>(
      "/ai_runtime/listen/event", k1muse_common::qos::ReliableEvent(10),
      std::bind(&MultimodalSupervisorNode::on_listen_event, this, std::placeholders::_1));

  listen_result_sub_ = create_subscription<ListenResult>(
      "/voice/listen/result", k1muse_common::qos::ReliableResult(10),
      std::bind(&MultimodalSupervisorNode::on_listen_result, this, std::placeholders::_1));

  intent_status_sub_ = create_subscription<IntentStatus>(
      "/voice/intent/status", k1muse_common::qos::LatchedState(),
      std::bind(&MultimodalSupervisorNode::on_intent_status, this, std::placeholders::_1));

  tts_status_sub_ = create_subscription<TtsStatus>(
      "/voice/tts/status", k1muse_common::qos::LatchedState(1),
      std::bind(&MultimodalSupervisorNode::on_tts_status, this, std::placeholders::_1));

  audio_playback_state_sub_ = create_subscription<AudioPlaybackState>(
      "/voice/audio/playback_state", k1muse_common::qos::LatchedState(1),
      std::bind(&MultimodalSupervisorNode::on_audio_playback_state, this, std::placeholders::_1));

  detection_2d_sub_ = create_subscription<Detection2DFrame>(
      "/vision/detection_2d", k1muse_common::qos::ReliableResult(5),
      std::bind(&MultimodalSupervisorNode::on_detection_2d, this, std::placeholders::_1));

  target_request_sub_ = create_subscription<TargetRequest>(
      "/vision/target_request", k1muse_common::qos::ReliableEvent(5),
      std::bind(&MultimodalSupervisorNode::on_target_request, this, std::placeholders::_1));

  audio_io_state_sub_ = create_subscription<NodeReady>(
      "/audio_io/state", k1muse_common::qos::LatchedState(),
      std::bind(&MultimodalSupervisorNode::on_audio_io_state, this, std::placeholders::_1));

  intent_state_sub_ = create_subscription<NodeReady>(
      "/intent/state", k1muse_common::qos::LatchedState(),
      std::bind(&MultimodalSupervisorNode::on_intent_state, this, std::placeholders::_1));

  // Publish initial control and interaction state (BOOT).
  publish_control();
  publish_interaction_state("boot");

  RCLCPP_INFO(get_logger(),
              "[startup] supervisor target_cache_ttl_ms=%d "
              "no_speech_timeout_sec=%.1f tts_completion_timeout_sec=%.1f "
              "topics={control_out:/ai_runtime/control audio_play_out:/voice/audio/play "
              "target_response_out:/vision/target_response "
              "interaction_state_out:/robot/interaction_state "
              "runtime_state_in:/ai_runtime/state "
              "wakeword_in:/ai_runtime/wakeword/event "
              "listen_result_in:/voice/listen/result "
              "detection_in:/vision/detection_2d "
              "target_request_in:/vision/target_request}",
              target_cache_ttl_ms_, no_speech_timeout_sec_,
              tts_completion_timeout_sec_);
}

MultimodalSupervisorNode::~MultimodalSupervisorNode() {
  cancel_no_speech_timer();
  cancel_wakeword_cooldown_timer();
  cancel_tts_completion_timer();
}

// ---------------------------------------------------------------------------
// Callback: AiRuntimeState  (runtime readiness)
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_runtime_state(AiRuntimeState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (msg->runtime_ready != runtime_ready_) {
    runtime_ready_ = msg->runtime_ready;
    RCLCPP_INFO(get_logger(), "Runtime ready changed: %s",
                runtime_ready_ ? "true" : "false");

    TransitionContext ctx;
    ctx.runtime_ready = runtime_ready_;
    ctx.audio_ready = audio_ready_;
    ctx.intent_ready = intent_ready_;
    auto result = state_machine_.process(Event::RUNTIME_READY_CHANGED, ctx);
    if (result.accepted) {
      transition_to(result.new_state, result.reason);
    }
  }
}

// ---------------------------------------------------------------------------
// Callback: WakewordEvent
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_wakeword_event(WakewordEvent::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_machine_.current_state() != State::IDLE) {
    RCLCPP_DEBUG(get_logger(), "Ignoring wakeword in state %s",
                 state_name(state_machine_.current_state()));
    return;
  }

  if (msg->event != WakewordEvent::EVENT_DETECTED) {
    return;
  }

  // Generate new interaction identity.
  generate_new_identity();

  // Generate request_id for the wake-ack audio play request.
  std::string req_id = k1muse_common::make_id("wake_ack");
  active_request_id_ = req_id;

  RCLCPP_INFO(get_logger(),
              "Wakeword detected: trace=%s epoch=%" PRIu64 " keyword=%s conf=%.2f",
              active_trace_id_.c_str(), active_epoch_,
              msg->keyword.c_str(), msg->confidence);

  // Publish AudioPlayRequest for wake acknowledgement.
  AudioPlayRequest play_msg;
  play_msg.header.stamp = now();
  play_msg.trace_id = active_trace_id_;
  play_msg.request_id = req_id;
  play_msg.epoch = active_epoch_;
  play_msg.source = "supervisor";
  play_msg.kind = AudioPlayRequest::KIND_PRESET;
  play_msg.preset_name = wake_ack_preset_;
  audio_play_publisher_->publish(play_msg);

  // Transition: IDLE -> WAKE_ACK.
  TransitionContext ctx;
  ctx.active_epoch = active_epoch_;
  ctx.event_epoch = active_epoch_;
  auto result = state_machine_.process(Event::WAKEWORD_ACCEPTED, ctx);
  if (result.accepted) {
    transition_to(result.new_state, result.reason);
  }
}

// ---------------------------------------------------------------------------
// Callback: ListenEvent  (informational, no state transition)
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_listen_event(ListenEvent::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  RCLCPP_DEBUG(get_logger(), "ListenEvent: trace=%s event=%s",
               msg->trace_id.c_str(), msg->event_name.c_str());
}

// ---------------------------------------------------------------------------
// Callback: ListenResult
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_listen_result(ListenResult::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_machine_.current_state() != State::LISTENING) {
    RCLCPP_DEBUG(get_logger(), "Ignoring ListenResult in state %s",
                 state_name(state_machine_.current_state()));
    return;
  }

  // Check trace_id and epoch.
  TransitionContext ctx;
  ctx.active_trace_id = active_trace_id_;
  ctx.active_epoch = active_epoch_;
  ctx.event_trace_id = msg->trace_id;
  ctx.event_epoch = msg->epoch;

  auto result = state_machine_.process(Event::LISTEN_RESULT_RECEIVED, ctx);
  if (!result.accepted) {
    RCLCPP_WARN(get_logger(), "ListenResult rejected: %s", result.reason.c_str());
    return;
  }

  // Update identity for intent phase: track utterance_id and intent's request_id.
  active_utterance_id_ = msg->utterance_id;

  cancel_no_speech_timer();

  RCLCPP_INFO(get_logger(),
              "[trace] listen_result accepted trace_id=%s utterance_id=%s "
              "epoch=%llu text_len=%zu",
              msg->trace_id.c_str(), msg->utterance_id.c_str(),
              static_cast<unsigned long long>(msg->epoch), msg->text.size());

  transition_to(result.new_state, result.reason);
}

// ---------------------------------------------------------------------------
// Callback: IntentStatus
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_intent_status(IntentStatus::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_machine_.current_state() != State::INTENT_PROCESSING) {
    RCLCPP_DEBUG(get_logger(), "Ignoring IntentStatus in state %s",
                 state_name(state_machine_.current_state()));
    return;
  }

  // Update active_request_id to the intent's request_id for TTS matching.
  TransitionContext ctx;
  ctx.active_trace_id = active_trace_id_;
  ctx.active_epoch = active_epoch_;
  ctx.active_request_id = msg->request_id;
  ctx.event_trace_id = msg->trace_id;
  ctx.event_epoch = msg->epoch;
  ctx.event_request_id = msg->request_id;

  if (msg->state == IntentStatus::STATE_FINISHED) {
    ctx.has_pending_tts = msg->has_tts;
    auto result = state_machine_.process(Event::INTENT_FINISHED, ctx);
    if (result.accepted) {
      active_request_id_ = msg->request_id;
      RCLCPP_INFO(get_logger(), "Intent finished: req=%s has_tts=%d -> %s",
                  msg->request_id.c_str(), msg->has_tts,
                  result.new_state == State::IDLE ? "IDLE" : "TTS_RUNNING");
      transition_to(result.new_state, result.reason);
    } else {
      RCLCPP_WARN(get_logger(), "INTENT_FINISHED rejected: %s", result.reason.c_str());
    }
  } else if (msg->state == IntentStatus::STATE_FAILED) {
    ctx.has_fallback_tts = false;
    auto result = state_machine_.process(Event::INTENT_FAILED, ctx);
    if (result.accepted) {
      RCLCPP_WARN(get_logger(), "Intent failed: %s", msg->reason.c_str());
      transition_to(result.new_state, result.reason);
    }
  }
}

// ---------------------------------------------------------------------------
// Callback: TtsStatus  (TTS inference completion)
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_tts_status(TtsStatus::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_machine_.current_state() != State::TTS_RUNNING) {
    return;
  }

  if (msg->state == TtsStatus::STATE_DONE) {
    TransitionContext ctx;
    ctx.active_epoch = active_epoch_;
    ctx.active_request_id = active_request_id_;
    ctx.event_epoch = msg->epoch;
    ctx.event_request_id = msg->request_id;

    auto result = state_machine_.process(Event::TTS_DONE, ctx);
    if (result.accepted) {
      RCLCPP_INFO(get_logger(), "TTS inference done + playback done -> IDLE");
      transition_to(result.new_state, result.reason);
    } else {
      RCLCPP_DEBUG(get_logger(), "TTS_DONE rejected (stale or wrong state)");
    }
  } else if (msg->state == TtsStatus::STATE_FAILED) {
    TransitionContext ctx;
    auto result = state_machine_.process(Event::TTS_FAILED, ctx);
    if (result.accepted) {
      RCLCPP_WARN(get_logger(), "TTS failed: %s", msg->reason.c_str());
      transition_to(result.new_state, result.reason);
    }
  }
}

// ---------------------------------------------------------------------------
// Callback: AudioPlaybackState  (playback completion for wake-ack or TTS)
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_audio_playback_state(
    AudioPlaybackState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto current = state_machine_.current_state();

  if (current == State::WAKE_ACK) {
    // Wake-ack playback completion.
    if (msg->state == AudioPlaybackState::STATE_DONE) {
      TransitionContext ctx;
      ctx.active_epoch = active_epoch_;
      ctx.active_request_id = active_request_id_;
      ctx.event_epoch = msg->epoch;
      ctx.event_request_id = msg->request_id;

      auto result = state_machine_.process(Event::WAKE_ACK_PLAYBACK_DONE, ctx);
      if (result.accepted) {
        RCLCPP_INFO(get_logger(), "Wake-ack playback done -> LISTENING");
        transition_to(result.new_state, result.reason);
        // Start no-speech timer now that we are LISTENING.
        start_no_speech_timer();
      } else {
        RCLCPP_WARN(get_logger(), "Wake-ack playback rejected: %s",
                     result.reason.c_str());
      }
    } else if (msg->state == AudioPlaybackState::STATE_FAILED) {
      TransitionContext ctx;
      auto result = state_machine_.process(Event::WAKE_ACK_PLAYBACK_FAILED, ctx);
      if (result.accepted) {
        RCLCPP_WARN(get_logger(), "Wake-ack playback failed (reason: %s), fallback to LISTENING",
                    msg->reason.c_str());
        transition_to(result.new_state, result.reason);
        start_no_speech_timer();
      }
    }
  } else if (current == State::TTS_RUNNING) {
    // TTS playback completion.
    if (msg->state == AudioPlaybackState::STATE_DONE) {
      TransitionContext ctx;
      ctx.active_epoch = active_epoch_;
      ctx.active_request_id = active_request_id_;
      ctx.event_epoch = msg->epoch;
      ctx.event_request_id = msg->request_id;

      auto result = state_machine_.process(Event::TTS_PLAYBACK_DONE, ctx);
      if (result.accepted) {
        RCLCPP_INFO(get_logger(), "TTS playback done + inference done -> IDLE");
        transition_to(result.new_state, result.reason);
      } else {
        RCLCPP_DEBUG(get_logger(), "TTS playback done, waiting for inference");
      }
    } else if (msg->state == AudioPlaybackState::STATE_FAILED) {
      TransitionContext ctx;
      auto result = state_machine_.process(Event::TTS_PLAYBACK_FAILED, ctx);
      if (result.accepted) {
        RCLCPP_WARN(get_logger(), "TTS playback failed (reason: %s)", msg->reason.c_str());
        transition_to(result.new_state, result.reason);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Callback: Detection2DFrame  (update target cache)
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_detection_2d(Detection2DFrame::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<CachedDetection> cached;
  cached.reserve(msg->detections.size());
  for (const auto& det : msg->detections) {
    CachedDetection cd;
    cd.detection_id = det.detection_id;
    cd.class_name = det.class_name;
    cd.score = det.score;
    cd.x = det.x;
    cd.y = det.y;
    cd.width = det.width;
    cd.height = det.height;
    cached.push_back(std::move(cd));
  }
  target_cache_->update(msg->image_width, msg->image_height,
                        std::move(cached), std::chrono::steady_clock::now());
}

// ---------------------------------------------------------------------------
// Callback: TargetRequest  (synchronous target lookup)
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_target_request(TargetRequest::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_machine_.current_state() != State::IDLE) {
    RCLCPP_DEBUG(get_logger(), "Ignoring TargetRequest in state %s",
                 state_name(state_machine_.current_state()));
    return;
  }
  RCLCPP_INFO(get_logger(),
              "[trace] target_request trace_id=%s request_id=%s epoch=%llu "
              "target_class=%s min_score=%.2f timeout_ms=%u",
              msg->trace_id.c_str(), msg->request_id.c_str(),
              static_cast<unsigned long long>(msg->epoch),
              msg->target_class.c_str(), msg->minimum_score,
              static_cast<unsigned>(msg->timeout_ms));

  // Transition IDLE -> TARGETING.
  TransitionContext ctx;
  auto result = state_machine_.process(Event::TARGET_REQUEST_RECEIVED, ctx);
  if (!result.accepted) {
    RCLCPP_WARN(get_logger(), "TARGET_REQUEST_RECEIVED rejected: %s",
                result.reason.c_str());
    return;
  }
  transition_to(result.new_state, result.reason);

  // Synchronous lookup from cache.
  TargetResponse resp;
  resp.header.stamp = now();
  resp.request_id = msg->request_id;
  resp.trace_id = msg->trace_id;
  resp.epoch = msg->epoch;

  try {
    auto tr = target_cache_->find_target(msg->target_class, msg->minimum_score);
    resp.found = tr.found;
    resp.target_id = tr.target_id;
    resp.target_class = tr.target_class;
    resp.score = tr.score;
    resp.reason = tr.reason;
  } catch (const std::exception& e) {
    resp.found = false;
    resp.reason = e.what();
  }

  target_response_publisher_->publish(resp);

  // Transition TARGETING -> IDLE.
  TransitionContext ctx2;
  auto result2 = state_machine_.process(Event::TARGET_RESPONSE_SENT, ctx2);
  if (result2.accepted) {
    transition_to(result2.new_state, result2.reason);
  }
}

// ---------------------------------------------------------------------------
// Callback: AudioIoNode readiness
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_audio_io_state(NodeReady::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (msg->ready != audio_ready_) {
    audio_ready_ = msg->ready;
    RCLCPP_INFO(get_logger(), "Audio ready changed: %s",
                audio_ready_ ? "true" : "false");

    TransitionContext ctx;
    ctx.runtime_ready = runtime_ready_;
    ctx.audio_ready = audio_ready_;
    ctx.intent_ready = intent_ready_;
    auto result = state_machine_.process(Event::AUDIO_READY_CHANGED, ctx);
    if (result.accepted) {
      transition_to(result.new_state, result.reason);
    }
  }
}

// ---------------------------------------------------------------------------
// Callback: IntentNode readiness
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::on_intent_state(NodeReady::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (msg->ready != intent_ready_) {
    intent_ready_ = msg->ready;
    RCLCPP_INFO(get_logger(), "Intent ready changed: %s",
                intent_ready_ ? "true" : "false");

    TransitionContext ctx;
    ctx.runtime_ready = runtime_ready_;
    ctx.audio_ready = audio_ready_;
    ctx.intent_ready = intent_ready_;
    auto result = state_machine_.process(Event::INTENT_READY_CHANGED, ctx);
    if (result.accepted) {
      transition_to(result.new_state, result.reason);
    }
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void MultimodalSupervisorNode::transition_to(State new_state,
                                              const std::string& reason) {
  const State previous_state = state_machine_.current_state();
  state_machine_.set_state(new_state);

  if (previous_state == State::TTS_RUNNING && new_state != State::TTS_RUNNING) {
    cancel_tts_completion_timer();
  }
  if (new_state == State::TTS_RUNNING && previous_state != State::TTS_RUNNING) {
    start_tts_completion_timer();
  }

  RCLCPP_INFO(get_logger(),
              "[trace] state_transition from=%s to=%s reason=%s "
              "trace_id=%s request_id=%s epoch=%llu",
              state_name(previous_state), state_name(new_state), reason.c_str(),
              active_trace_id_.c_str(), active_request_id_.c_str(),
              static_cast<unsigned long long>(active_epoch_));
  publish_control();
  publish_interaction_state(reason);
}

void MultimodalSupervisorNode::publish_control() {
  auto flags = RuntimeControlBuilder::flags_for_state(
      state_machine_.current_state());

  AiRuntimeControl msg;
  msg.header.stamp = now();
  msg.trace_id = active_trace_id_;
  msg.epoch = active_epoch_;
  msg.interaction_state =
      static_cast<uint8_t>(state_machine_.current_state());
  msg.interaction_state_name = state_name(state_machine_.current_state());
  msg.wakeword_enabled = flags.wakeword_enabled;
  msg.vision_enabled = flags.vision_enabled;
  msg.vad_asr_enabled = flags.vad_asr_enabled;
  msg.tts_enabled = flags.tts_enabled;
  msg.reason = "";
  control_publisher_->publish(msg);
}

void MultimodalSupervisorNode::publish_interaction_state(
    const std::string& reason) {
  auto flags = RuntimeControlBuilder::flags_for_state(
      state_machine_.current_state());

  InteractionState msg;
  msg.header.stamp = now();
  msg.trace_id = active_trace_id_;
  msg.epoch = active_epoch_;
  msg.state = static_cast<uint8_t>(state_machine_.current_state());
  msg.state_name = state_name(state_machine_.current_state());
  msg.wakeword_enabled = flags.wakeword_enabled;
  msg.vision_enabled = flags.vision_enabled;
  msg.vad_asr_enabled = flags.vad_asr_enabled;
  msg.tts_enabled = flags.tts_enabled;
  msg.active_request_id = active_request_id_;
  msg.active_utterance_id = active_utterance_id_;
  msg.reason = reason;
  interaction_state_publisher_->publish(msg);
}

void MultimodalSupervisorNode::start_no_speech_timer() {
  cancel_no_speech_timer();
  no_speech_timer_ = create_wall_timer(
      std::chrono::milliseconds(
          static_cast<int>(no_speech_timeout_sec_ * 1000.0)),
      [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cancel_no_speech_timer();
        if (state_machine_.current_state() != State::LISTENING) {
          return;
        }
        RCLCPP_WARN(get_logger(), "No-speech timeout (%.1fs)",
                     no_speech_timeout_sec_);
        TransitionContext ctx;
        auto result = state_machine_.process(Event::NO_SPEECH_TIMEOUT, ctx);
        if (result.accepted) {
          transition_to(result.new_state, result.reason);
          // Suppress wakeword for a brief cooldown so residual audio
          // does not immediately re-trigger IDLE→WAKE_ACK→LISTENING.
          publish_wakeword_cooldown();
        }
      });
}

void MultimodalSupervisorNode::cancel_no_speech_timer() {
  if (no_speech_timer_) {
    no_speech_timer_->cancel();
    no_speech_timer_.reset();
  }
}

void MultimodalSupervisorNode::start_tts_completion_timer() {
  cancel_tts_completion_timer();
  if (tts_completion_timeout_sec_ <= 0.0) {
    return;
  }

  const auto timeout = std::chrono::milliseconds(
      std::max(1, static_cast<int>(tts_completion_timeout_sec_ * 1000.0)));
  tts_completion_timer_ = create_wall_timer(
      timeout,
      [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cancel_tts_completion_timer();
        if (state_machine_.current_state() != State::TTS_RUNNING) {
          return;
        }

        RCLCPP_WARN(get_logger(), "TTS completion timeout (%.1fs)",
                    tts_completion_timeout_sec_);
        TransitionContext ctx;
        auto result = state_machine_.process(Event::TTS_FAILED, ctx);
        if (result.accepted) {
          transition_to(result.new_state, "tts completion timeout");
        }
      });
}

void MultimodalSupervisorNode::cancel_tts_completion_timer() {
  if (tts_completion_timer_) {
    tts_completion_timer_->cancel();
    tts_completion_timer_.reset();
  }
}

void MultimodalSupervisorNode::publish_wakeword_cooldown() {
  // Immediately disable wakeword so residual audio does not re-trigger.
  AiRuntimeControl cooldown_msg;
  cooldown_msg.header.stamp = now();
  cooldown_msg.trace_id = active_trace_id_;
  cooldown_msg.epoch = active_epoch_;
  cooldown_msg.interaction_state = static_cast<uint8_t>(State::IDLE);
  cooldown_msg.interaction_state_name = "IDLE";
  cooldown_msg.wakeword_enabled = false;
  cooldown_msg.vision_enabled = true;
  cooldown_msg.vad_asr_enabled = false;
  cooldown_msg.tts_enabled = false;
  cooldown_msg.reason = "no-speech timeout cooldown";
  control_publisher_->publish(cooldown_msg);

  // Re-enable wakeword after 500 ms.
  cancel_wakeword_cooldown_timer();
  wakeword_cooldown_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        publish_control();
        wakeword_cooldown_timer_->cancel();
        wakeword_cooldown_timer_.reset();
      });
}

void MultimodalSupervisorNode::cancel_wakeword_cooldown_timer() {
  if (wakeword_cooldown_timer_) {
    wakeword_cooldown_timer_->cancel();
    wakeword_cooldown_timer_.reset();
  }
}

void MultimodalSupervisorNode::generate_new_identity() {
  // Cancel any pending cooldown; a new wakeword means a fresh cycle.
  cancel_wakeword_cooldown_timer();
  active_trace_id_ = k1muse_common::make_id("trace");
  ++active_epoch_;
  if (active_epoch_ == 0) {
    active_epoch_ = 1;  // 0 is sentinel for "no epoch"
  }
  active_request_id_.clear();
  active_utterance_id_.clear();
}

}  // namespace k1muse_multimodal_supervisor
