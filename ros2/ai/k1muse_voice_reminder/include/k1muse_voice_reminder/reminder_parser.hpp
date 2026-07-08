#pragma once

#include <string>

namespace k1muse_voice_reminder {

struct ParsedReminder {
  bool ok = false;
  std::string error;
  std::string trace_id;
  std::string text;
  std::string run_at_iso;
  std::string timezone = "Asia/Shanghai";
};

class ReminderParser {
 public:
  explicit ReminderParser(std::string timezone);

  ParsedReminder ParseCreateIntent(
      const std::string& trace_id,
      const std::string& text,
      const std::string& value) const;

 private:
  std::string timezone_;
};

std::string FormatShanghaiIsoFromNow(int delay_seconds);
std::string MakeReminderId(const std::string& trace_id, const std::string& text);
std::string DefaultReminderSpeech(const std::string& text);

}  // namespace k1muse_voice_reminder
