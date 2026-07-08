#include "k1muse_voice_reminder/reminder_node.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <sstream>
#include <utility>

#include "k1muse_common/qos_profiles.hpp"
#include "k1muse_voice_reminder/reminder_parser.hpp"

namespace k1muse_voice_reminder {

ReminderNode::ReminderNode(const rclcpp::NodeOptions& options)
    : Node("reminder_node", options) {
  // Parameters
  timezone_ = declare_parameter("timezone", std::string{"Asia/Shanghai"});
  timer_period_sec_ = declare_parameter("timer_period_sec", 1.0);
  const auto db_path = declare_parameter(
      "database_path", std::string{"reminders.sqlite3"});

  // Store
  store_ = std::make_unique<ReminderStore>(db_path);
  if (!store_->Open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open reminder database: %s",
                 store_->last_error().c_str());
    throw std::runtime_error("Failed to open reminder database: " + store_->last_error());
  }

  // Handlers
  intent_handler_ = std::make_unique<ReminderIntentHandler>(store_.get(), timezone_);
  due_handler_ = std::make_unique<ReminderDueHandler>(this);

  // Publishers
  tts_publisher_ = create_publisher<TtsTextRequest>(
      "/voice/tts/text", k1muse_common::qos::ReliableEvent(5));

  // Subscriptions
  intent_subscription_ = create_subscription<IntentLite>(
      "/voice/intent", k1muse_common::qos::ReliableResult(10),
      std::bind(&ReminderNode::on_intent, this, std::placeholders::_1));

  // Timer
  timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_sec_),
      std::bind(&ReminderNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "ReminderNode started (timer=%.1fs, tz=%s, db=%s)",
              timer_period_sec_, timezone_.c_str(), db_path.c_str());
}

void ReminderNode::Publish(const TtsTextRequest& request) {
  tts_publisher_->publish(request);
}

void ReminderNode::on_intent(IntentLite::SharedPtr msg) {
  auto result = intent_handler_->HandleIntent(*msg);
  if (result.ignored) {
    return;
  }
  if (!result.error.empty()) {
    RCLCPP_WARN(get_logger(), "Reminder intent error: %s", result.error.c_str());
    return;
  }
  // If there's a TTS reply (cancel/list), publish it.
  if (!result.tts_reply.empty()) {
    TtsTextRequest tts_msg;
    tts_msg.header.stamp = now();
    tts_msg.trace_id = msg->trace_id;
    tts_msg.request_id = msg->request_id;
    tts_msg.epoch = msg->epoch;
    tts_msg.source = "reminder_node";
    tts_msg.priority = TtsTextRequest::PRIORITY_REMINDER;
    tts_msg.text = result.tts_reply;
    tts_publisher_->publish(tts_msg);
  }
  if (result.created) {
    RCLCPP_INFO(get_logger(), "Reminder created (trace=%s)", msg->trace_id.c_str());
  }
}

void ReminderNode::on_timer() {
  // Build current Shanghai time ISO string.
  const auto now_tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &now_tt);
#else
  localtime_r(&now_tt, &tm);
#endif
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << "+08:00";
  const auto now_iso = os.str();

  auto due = store_->LoadDue(now_iso);
  for (const auto& record : due) {
    RCLCPP_INFO(get_logger(), "Reminder due: %s (id=%s)", record.text.c_str(), record.id.c_str());
    // Mark fired BEFORE publishing TTS so a subsequent timer tick
    // (unlikely at 1 s period, but defensive) won't re-fire the same record.
    store_->MarkFired(record.id, now_iso);
    const auto speech = DefaultReminderSpeech(record.text);
    due_handler_->OnReminderDue(record.trace_id, record.request_id, 0, speech);
  }
}

}  // namespace k1muse_voice_reminder
