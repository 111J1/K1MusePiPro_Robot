#pragma once

#include <string>

#include "k1muse_voice_msgs/msg/intent_lite.hpp"
#include "k1muse_voice_msgs/msg/tts_text_request.hpp"
#include "k1muse_voice_reminder/reminder_parser.hpp"
#include "k1muse_voice_reminder/reminder_store.hpp"

namespace k1muse_voice_reminder {

struct ReminderIntentResult {
  bool created = false;
  bool cancelled = false;
  bool listed = false;
  bool ignored = false;
  std::string tts_reply;
  std::string error;
};

class ITtsTextPublisher {
 public:
  virtual ~ITtsTextPublisher() = default;
  virtual void Publish(const k1muse_voice_msgs::msg::TtsTextRequest& request) = 0;
};

class ReminderIntentHandler {
 public:
  ReminderIntentHandler(IReminderStore* store, std::string timezone);

  ReminderIntentResult HandleIntent(const k1muse_voice_msgs::msg::IntentLite& intent);

 private:
  IReminderStore* store_;
  ReminderParser parser_;
};

class ReminderDueHandler {
 public:
  explicit ReminderDueHandler(ITtsTextPublisher* publisher);

  void OnReminderDue(
      const std::string& trace_id,
      const std::string& request_id,
      uint64_t epoch,
      const std::string& speech_text);

 private:
  ITtsTextPublisher* publisher_;
};

}  // namespace k1muse_voice_reminder
