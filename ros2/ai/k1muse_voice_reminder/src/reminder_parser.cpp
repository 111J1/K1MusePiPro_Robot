#include "k1muse_voice_reminder/reminder_parser.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace k1muse_voice_reminder {
namespace {

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

int ParseDelaySeconds(const std::string& value) {
  if (StartsWith(value, "PT") && value.size() >= 4) {
    const auto unit = value.back();
    const auto number = std::stoi(value.substr(2, value.size() - 3));
    if (unit == 'S') {
      return number;
    }
    if (unit == 'M') {
      return number * 60;
    }
    if (unit == 'H') {
      return number * 3600;
    }
  }
  const auto minute_pos = value.find("minutes later");
  if (minute_pos != std::string::npos) {
    return std::stoi(value.substr(0, minute_pos)) * 60;
  }
  const auto second_pos = value.find("seconds later");
  if (second_pos != std::string::npos) {
    return std::stoi(value.substr(0, second_pos));
  }
  return -1;
}

std::tm LocalTime(std::time_t time) {
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &time);
#else
  localtime_r(&time, &tm);
#endif
  return tm;
}

// NOTE: This function assumes the system timezone is Asia/Shanghai (UTC+8).
// The robot is deployed in China, so this is acceptable for Phase 1.
// For multi-timezone support, use proper timezone conversion libraries.
std::string TimeTToShanghaiIso(std::time_t tt) {
  const auto tm = LocalTime(tt);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << "+08:00";
  return os.str();
}

int ChineseDigitValue(const std::string& ch) {
  if (ch == "零") return 0;
  if (ch == "一" || ch == "壹") return 1;
  if (ch == "二" || ch == "两" || ch == "俩") return 2;
  if (ch == "三" || ch == "叁") return 3;
  if (ch == "四" || ch == "肆") return 4;
  if (ch == "五" || ch == "伍") return 5;
  if (ch == "六" || ch == "陆") return 6;
  if (ch == "七" || ch == "柒") return 7;
  if (ch == "八" || ch == "捌") return 8;
  if (ch == "九" || ch == "玖") return 9;
  if (ch == "十") return 10;
  return -1;
}

int ParseChineseInteger(const std::string& text) {
  if (text.empty()) return -1;
  // Single digit (3-byte UTF-8 char)
  if (text.size() == 3) {
    int d = ChineseDigitValue(text);
    if (d >= 0 && d <= 9) return d;
    if (d == 10) return 10;
    return -1;
  }
  // "十X" pattern: starts with 十
  if (text.size() >= 6 && text.substr(0, 3) == "十") {
    if (text.size() == 6) {
      int d = ChineseDigitValue(text.substr(3, 3));
      if (d >= 0 && d <= 9) return 10 + d;
    }
    return 10;
  }
  // "X十" or "X十Y" pattern
  if (text.size() >= 6) {
    int tens = ChineseDigitValue(text.substr(0, 3));
    if (tens >= 1 && tens <= 9 && text.substr(3, 3) == "十") {
      if (text.size() == 6) return tens * 10;
      if (text.size() == 9) {
        int ones = ChineseDigitValue(text.substr(6, 3));
        if (ones >= 0 && ones <= 9) return tens * 10 + ones;
      }
    }
  }
  return -1;
}

int ParseChineseDelay(const std::string& text) {
  // Try to find Chinese time units
  static const std::vector<std::pair<std::string, int>> units = {
      {"小时", 3600}, {"分钟", 60}, {"分", 60}, {"秒", 1},
  };
  for (const auto& [unit_str, multiplier] : units) {
    auto pos = text.find(unit_str);
    if (pos == std::string::npos) continue;
    std::string num_part = text.substr(0, pos);
    if (num_part.empty()) continue;
    // Try Chinese integer first
    int val = ParseChineseInteger(num_part);
    if (val < 0) {
      // Try Arabic digits
      try {
        val = std::stoi(num_part);
      } catch (...) {
        continue;
      }
    }
    if (val > 0) return val * multiplier;
  }
  return -1;
}

bool ParseCalendarTime(const std::string& text, std::time_t* run_at) {
  const auto now = std::time(nullptr);
  auto tm = LocalTime(now);

  bool is_tomorrow = false;
  int hour = -1;
  int minute = 0;

  if (text.find("明天") != std::string::npos) {
    is_tomorrow = true;
  }

  if (text.find("早上") != std::string::npos || text.find("上午") != std::string::npos) {
    hour = 8;
  } else if (text.find("下午") != std::string::npos) {
    hour = 15;
  } else if (text.find("晚上") != std::string::npos) {
    hour = 20;
  } else if (text.find("中午") != std::string::npos && is_tomorrow) {
    // "明天中午" — schedule for tomorrow 12:00
    hour = 12;
  } else if (is_tomorrow) {
    // Just "明天" with no time-of-day
    hour = 9;
  }

  if (hour < 0) {
    // Bare "中午" without "明天" — schedule for today or tomorrow depending on current time
    if (text.find("中午") != std::string::npos) {
      hour = 12;
      if (tm.tm_hour >= 12) {
        is_tomorrow = true;
      }
    } else {
      return false;
    }
  }

  if (is_tomorrow) {
    tm.tm_mday += 1;
  }
  tm.tm_hour = hour;
  tm.tm_min = minute;
  tm.tm_sec = 0;
  tm.tm_isdst = -1;

  *run_at = std::mktime(&tm);
  return *run_at != static_cast<std::time_t>(-1);
}

}  // namespace

ReminderParser::ReminderParser(std::string timezone) : timezone_(std::move(timezone)) {}

ParsedReminder ReminderParser::ParseCreateIntent(
    const std::string& trace_id,
    const std::string& text,
    const std::string& value) const {
  ParsedReminder out;
  out.trace_id = trace_id;
  out.text = text;
  out.timezone = timezone_;
  if (text.empty()) {
    out.error = "reminder text is empty";
    return out;
  }
  // 1. Try ISO 8601 with +08:00
  if (value.find('T') != std::string::npos && value.find("+08:00") != std::string::npos) {
    out.run_at_iso = value;
    out.ok = true;
    return out;
  }
  // 2. Try PT/English delay
  const auto delay_seconds = ParseDelaySeconds(value);
  if (delay_seconds > 0) {
    out.run_at_iso = FormatShanghaiIsoFromNow(delay_seconds);
    out.ok = true;
    return out;
  }
  // 3. Try Chinese delay (e.g., "五分钟", "5分钟", "30秒")
  const auto cn_delay = ParseChineseDelay(value);
  if (cn_delay > 0) {
    out.run_at_iso = FormatShanghaiIsoFromNow(cn_delay);
    out.ok = true;
    return out;
  }
  // 4. Try calendar time (e.g., "明天下午", "中午")
  std::time_t run_at = 0;
  if (ParseCalendarTime(value, &run_at)) {
    out.run_at_iso = TimeTToShanghaiIso(run_at);
    out.ok = true;
    return out;
  }
  // 5. No time expression found — default to 5 minutes from now.
  //    This handles bare "提醒我喝水" where only content is given.
  out.run_at_iso = FormatShanghaiIsoFromNow(300);
  out.ok = true;
  return out;
}

std::string FormatShanghaiIsoFromNow(int delay_seconds) {
  const auto now = std::chrono::system_clock::now() + std::chrono::seconds(delay_seconds);
  const auto tt = std::chrono::system_clock::to_time_t(now);
  const auto tm = LocalTime(tt);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << "+08:00";
  return os.str();
}

std::string MakeReminderId(const std::string& trace_id, const std::string& text) {
  const auto hash = std::hash<std::string>{}(trace_id + ":" + text);
  return "reminder-" + std::to_string(hash);
}

std::string DefaultReminderSpeech(const std::string& text) {
  return text.empty() ? "提醒时间到了。" : "提醒您：" + text;
}

}  // namespace k1muse_voice_reminder
