#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_voice_msgs/msg/intent_lite.hpp"
#include "k1muse_voice_msgs/msg/tts_text_request.hpp"
#include "k1muse_voice_reminder/reminder_handler.hpp"
#include "k1muse_voice_reminder/reminder_store.hpp"

namespace k1muse_voice_reminder {

class ReminderNode : public rclcpp::Node, public ITtsTextPublisher {
 public:
  using IntentLite = k1muse_voice_msgs::msg::IntentLite;
  using TtsTextRequest = k1muse_voice_msgs::msg::TtsTextRequest;

  explicit ReminderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // ITtsTextPublisher
  void Publish(const TtsTextRequest& request) override;

 private:
  void on_intent(IntentLite::SharedPtr msg);
  void on_timer();

  // Publishers
  rclcpp::Publisher<TtsTextRequest>::SharedPtr tts_publisher_;

  // Subscriptions
  rclcpp::Subscription<IntentLite>::SharedPtr intent_subscription_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Store and handlers
  std::unique_ptr<ReminderStore> store_;
  std::unique_ptr<ReminderIntentHandler> intent_handler_;
  std::unique_ptr<ReminderDueHandler> due_handler_;

  // Config
  std::string timezone_;
  double timer_period_sec_;
};

}  // namespace k1muse_voice_reminder
