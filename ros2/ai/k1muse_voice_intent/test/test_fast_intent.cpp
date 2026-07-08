#include <gtest/gtest.h>

#include "k1muse_voice_intent/fast_intent.hpp"

TEST(FastIntent, MoveForward) {
  const auto result = k1muse_voice_intent::MatchFastIntent("前进");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "move");
  EXPECT_EQ(result.target, "chassis");
  EXPECT_EQ(result.value, "forward");
  EXPECT_FALSE(result.tts_reply.empty());
  EXPECT_FALSE(result.requires_confirmation);
}

TEST(FastIntent, MoveBackward) {
  const auto result = k1muse_voice_intent::MatchFastIntent("后退");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "move");
  EXPECT_EQ(result.target, "chassis");
  EXPECT_EQ(result.value, "backward");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, TurnLeft) {
  const auto result = k1muse_voice_intent::MatchFastIntent("左转");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "move");
  EXPECT_EQ(result.target, "chassis");
  EXPECT_EQ(result.value, "left");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, TurnRight) {
  const auto result = k1muse_voice_intent::MatchFastIntent("右转");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "move");
  EXPECT_EQ(result.target, "chassis");
  EXPECT_EQ(result.value, "right");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, Stop) {
  const auto result = k1muse_voice_intent::MatchFastIntent("停止");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "stop");
  EXPECT_EQ(result.target, "chassis");
  EXPECT_FALSE(result.requires_confirmation);
}

TEST(FastIntent, Rotate) {
  const auto result = k1muse_voice_intent::MatchFastIntent("旋转");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "rotate");
  EXPECT_EQ(result.target, "chassis");
  EXPECT_EQ(result.value, "360");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, LiftUp) {
  const auto result = k1muse_voice_intent::MatchFastIntent("升高");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "lift");
  EXPECT_EQ(result.target, "lift");
  EXPECT_EQ(result.value, "up");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, LiftDown) {
  const auto result = k1muse_voice_intent::MatchFastIntent("降低");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "lift");
  EXPECT_EQ(result.target, "lift");
  EXPECT_EQ(result.value, "down");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, Introduce) {
  const auto result = k1muse_voice_intent::MatchFastIntent("你是谁");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "query");
  EXPECT_EQ(result.action, "introduce");
  EXPECT_EQ(result.target, "self");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, CapabilityQuestion) {
  const auto result = k1muse_voice_intent::MatchFastIntent("你能做什么");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "query");
  EXPECT_EQ(result.action, "introduce");
  EXPECT_EQ(result.target, "self");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, ChatGreeting) {
  const auto result = k1muse_voice_intent::MatchFastIntent("你好");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "chat");
  EXPECT_EQ(result.action, "chat");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, Time) {
  const auto result = k1muse_voice_intent::MatchFastIntent("几点");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "query");
  EXPECT_EQ(result.action, "time");
  EXPECT_EQ(result.target, "system");
}

TEST(FastIntent, BareReminderDoesNotCreateEmptyReminder) {
  const auto result = k1muse_voice_intent::MatchFastIntent("提醒我");
  EXPECT_FALSE(result.matched);
}

TEST(FastIntent, ReminderCreateWithContent) {
  const auto result = k1muse_voice_intent::MatchFastIntent("三分钟后提醒我关火");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "reminder");
  EXPECT_EQ(result.action, "create");
  EXPECT_EQ(result.target, "三分钟后关火");
  EXPECT_EQ(result.value, "三分钟后关火");
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, ReminderList) {
  const auto result = k1muse_voice_intent::MatchFastIntent("查看提醒");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "reminder");
  EXPECT_EQ(result.action, "list");
  EXPECT_EQ(result.target, "reminder");
}

TEST(FastIntent, Shutdown) {
  const auto result = k1muse_voice_intent::MatchFastIntent("关机");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "system");
  EXPECT_EQ(result.action, "shutdown");
  EXPECT_EQ(result.target, "system");
  EXPECT_TRUE(result.requires_confirmation);
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, Reboot) {
  const auto result = k1muse_voice_intent::MatchFastIntent("重启");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "system");
  EXPECT_EQ(result.action, "reboot");
  EXPECT_EQ(result.target, "system");
  EXPECT_TRUE(result.requires_confirmation);
  EXPECT_FALSE(result.tts_reply.empty());
}

TEST(FastIntent, FindTarget) {
  const auto result = k1muse_voice_intent::MatchFastIntent("找杯子");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "find");
  EXPECT_EQ(result.target, "杯子");
  EXPECT_FLOAT_EQ(result.confidence, 0.9f);
}

TEST(FastIntent, NoMatch) {
  const auto result = k1muse_voice_intent::MatchFastIntent("今天天气");
  EXPECT_FALSE(result.matched);
}

TEST(FastIntent, FindWithPrefix) {
  const auto result = k1muse_voice_intent::MatchFastIntent("帮我拿遥控器");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "find");
  EXPECT_EQ(result.target, "遥控器");
}
TEST(FastIntent, FindWithTakeSuffix) {
  const auto result = k1muse_voice_intent::MatchFastIntent("拿杯子过来");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "find");
  EXPECT_EQ(result.target, "杯子");
}

TEST(FastIntent, FindWithSearch) {
  const auto result = k1muse_voice_intent::MatchFastIntent("搜索钥匙");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "action");
  EXPECT_EQ(result.action, "find");
  EXPECT_EQ(result.target, "钥匙");
}

TEST(FastIntent, RebootChineseLong) {
  const auto result = k1muse_voice_intent::MatchFastIntent("重新启动");
  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.intent_name, "system");
  EXPECT_EQ(result.action, "reboot");
  EXPECT_EQ(result.target, "system");
  EXPECT_TRUE(result.requires_confirmation);
}
