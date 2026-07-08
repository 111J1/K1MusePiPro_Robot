#include <gtest/gtest.h>

#include "k1muse_voice_reminder/reminder_parser.hpp"

TEST(ReminderParser, ParsesIsoCreateIntentUsingShanghaiTimezone) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result =
      parser.ParseCreateIntent("trace-1", "喝水", "2026-06-15T20:00:00+08:00");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.timezone, "Asia/Shanghai");
  EXPECT_EQ(result.text, "喝水");
  EXPECT_EQ(result.run_at_iso, "2026-06-15T20:00:00+08:00");
}

TEST(ReminderParser, ParsesRelativeDelayUsingShanghaiTimezone) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "drink water", "PT10M");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.timezone, "Asia/Shanghai");
  EXPECT_EQ(result.text, "drink water");
  EXPECT_NE(result.run_at_iso.find("+08:00"), std::string::npos);
}

TEST(ReminderParser, ParsesChineseDelayFiveMinutes) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "五分钟");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("+08:00"), std::string::npos);
}

TEST(ReminderParser, ParsesChineseDelay30Seconds) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "30秒");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("+08:00"), std::string::npos);
}

TEST(ReminderParser, ParsesChineseDelayTwoHours) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "两小时");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("+08:00"), std::string::npos);
}

TEST(ReminderParser, ParsesChineseDelayFifteenMinutes) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "十五分钟");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("+08:00"), std::string::npos);
}

TEST(ReminderParser, ParsesCalendarTomorrowMorning) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "明天早上");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("08:00:00"), std::string::npos);
  EXPECT_NE(result.run_at_iso.find("+08:00"), std::string::npos);
}

TEST(ReminderParser, ParsesCalendarTomorrowAfternoon) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "明天下午");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("15:00:00"), std::string::npos);
}

TEST(ReminderParser, ParsesCalendarTomorrow) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "明天");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("09:00:00"), std::string::npos);
}

TEST(ReminderParser, ParsesCalendarNoon) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "中午");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NE(result.run_at_iso.find("12:00:00"), std::string::npos);
}

TEST(ReminderParser, RejectsEmptyText) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "", "PT5M");

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST(ReminderParser, RejectsUnsupportedExpression) {
  k1muse_voice_reminder::ReminderParser parser("Asia/Shanghai");

  const auto result = parser.ParseCreateIntent("trace-1", "喝水", "foobar");

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST(ReminderParser, MakeReminderIdIsStable) {
  const auto id1 = k1muse_voice_reminder::MakeReminderId("trace-1", "喝水");
  const auto id2 = k1muse_voice_reminder::MakeReminderId("trace-1", "喝水");
  EXPECT_EQ(id1, id2);
  EXPECT_TRUE(id1.rfind("reminder-", 0) == 0);
}

TEST(ReminderParser, DefaultReminderSpeechNonEmpty) {
  EXPECT_EQ(k1muse_voice_reminder::DefaultReminderSpeech("喝水"), "提醒您：喝水");
  EXPECT_EQ(k1muse_voice_reminder::DefaultReminderSpeech(""), "提醒时间到了。");
}
