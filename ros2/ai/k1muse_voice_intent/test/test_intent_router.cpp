#include <gtest/gtest.h>

#include <memory>

#include "k1muse_voice_intent/intent_router.hpp"
#include "k1muse_voice_intent/mock_llm_intent_client.hpp"

namespace k1muse_voice_intent {
namespace {

// Helper: create a default router with a fresh MockLlmIntentClient.
std::unique_ptr<IntentRouter> MakeDefaultRouter() {
  RouterConfig config;
  auto mock = std::make_unique<MockLlmIntentClient>();
  return std::make_unique<IntentRouter>(config, std::move(mock));
}

// Helper: create a router with custom config.
std::unique_ptr<IntentRouter> MakeRouter(RouterConfig config) {
  auto mock = std::make_unique<MockLlmIntentClient>();
  return std::make_unique<IntentRouter>(config, std::move(mock));
}

// --- Test 1: FastHitShortText ---
// "前进" matches fast intent, text is short (2 chars <= 10), LLM never called.
TEST(IntentRouterTest, FastHitShortText) {
  auto router = MakeDefaultRouter();
  auto result = router->process("前进");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.from_fast_intent);
  EXPECT_FALSE(result.llm_called);
  EXPECT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "action");
  EXPECT_EQ(result.decision.action, "move");
  EXPECT_EQ(result.decision.target, "chassis");
  EXPECT_EQ(result.decision.value, "forward");
}

// --- Test 2: FastHitSkippedComplexText ---
// A long text (> 10 chars) that would match fast intent, but because it's
// long, the router skips fast_intent and calls LLM instead.  The mock
// default returns "query_introduce" JSON.
TEST(IntentRouterTest, FastHitSkippedComplexText) {
  auto router = MakeDefaultRouter();
  auto result = router->process("请帮我前进到大厅的最前方");

  EXPECT_FALSE(result.failed);
  EXPECT_FALSE(result.from_fast_intent);
  EXPECT_TRUE(result.llm_called);
  EXPECT_TRUE(result.decision.matched);
  // Mock default JSON {"kind":"query_introduce"} maps to intent_name="query"
  EXPECT_EQ(result.decision.intent_name, "query");
  EXPECT_EQ(result.decision.action, "introduce");
}
TEST(IntentRouterTest, LongStopRoutesFastWithoutLlm) {
  auto router = MakeDefaultRouter();
  auto result = router->process("请你现在马上不要动");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.from_fast_intent);
  EXPECT_FALSE(result.llm_called);
  EXPECT_EQ(result.route_source, "fast");
  EXPECT_EQ(result.fast_category, "safety_fast");
  EXPECT_EQ(result.decision.intent_name, "action");
  EXPECT_EQ(result.decision.action, "stop");
}

TEST(IntentRouterTest, BareReminderDoesNotCreateEmptyReminder) {
  auto router = MakeDefaultRouter();
  auto result = router->process("提醒我");

  EXPECT_FALSE(result.failed);
  EXPECT_FALSE(result.llm_called);
  EXPECT_EQ(result.route_source, "safe_fallback");
  EXPECT_EQ(result.fast_slot_state, "slot_missing");
  EXPECT_EQ(result.decision.intent_name, "unknown");
  EXPECT_EQ(result.decision.action, "ask_repeat");
}
TEST(IntentRouterTest, ComplexMoveDoesNotExecuteRelativeMoveEvenIfLlmSaysMove) {
  RouterConfig config;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_default_json(
      R"({"kind":"move","direction":"forward","target":"","reply":"好的，向前走。"})");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("请帮我前进到大厅的最前方");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.llm_called);
  EXPECT_EQ(result.route_source, "safe_fallback");
  EXPECT_EQ(result.fast_reject_reason, "simple_action_too_complex");
  EXPECT_EQ(result.decision.intent_name, "unknown");
  EXPECT_EQ(result.decision.action, "ask_repeat");
}

// --- Test 3: FastMissLlmHit ---
// "今天天气不错" doesn't match fast intent, falls through to LLM which
// returns the default mock JSON.
TEST(IntentRouterTest, FastMissLlmHit) {
  auto router = MakeDefaultRouter();
  auto result = router->process("今天天气不错");

  EXPECT_FALSE(result.failed);
  EXPECT_FALSE(result.from_fast_intent);
  EXPECT_TRUE(result.llm_called);
  EXPECT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "query");
  EXPECT_EQ(result.decision.action, "introduce");
}

// --- Test 4: Empty LLM reply should not fail valid classification ---
TEST(IntentRouterTest, EmptyLlmReplyUsesLocalReplyFallback) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_default_json(
      R"({"kind":"query_introduce","direction":"","target":"","reply":""})");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("你能做什么");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.llm_called);
  EXPECT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "query");
  EXPECT_EQ(result.decision.action, "introduce");
  EXPECT_FALSE(result.decision.tts_reply.empty());
}

TEST(IntentRouterTest, MissingLlmReplyUsesLocalReplyFallback) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_default_json(
      R"({"kind":"query_introduce","direction":"","target":""})");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("你会什么");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "query");
  EXPECT_EQ(result.decision.action, "introduce");
  EXPECT_FALSE(result.decision.tts_reply.empty());
}

// --- Test 5: Invalid LLM response should recover safely ---
TEST(IntentRouterTest, InvalidLlmResponseUsesSafeFallback) {
  auto router = MakeDefaultRouter();
  auto result = router->process("未知");

  EXPECT_FALSE(result.failed);
  EXPECT_FALSE(result.from_fast_intent);
  EXPECT_TRUE(result.llm_called);
  ASSERT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "unknown");
  EXPECT_EQ(result.decision.action, "ask_repeat");
  EXPECT_FALSE(result.decision.tts_reply.empty());
}

TEST(IntentRouterTest, NoJsonLlmOutputUsesSafeFallback) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_default_json("我可以帮你移动、找东西和设置提醒。");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("你能做什么");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.llm_called);
  ASSERT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "unknown");
  EXPECT_EQ(result.decision.action, "ask_repeat");
  EXPECT_FALSE(result.decision.tts_reply.empty());
}

// --- Test 6: EmptyText ---
TEST(IntentRouterTest, EmptyText) {
  auto router = MakeDefaultRouter();
  auto result = router->process("");

  EXPECT_TRUE(result.failed);
  EXPECT_FALSE(result.from_fast_intent);
  EXPECT_FALSE(result.llm_called);
  EXPECT_EQ(result.failure_reason, "empty text");
}

// --- Test 6: LlmDisabled ---
TEST(IntentRouterTest, LlmDisabled) {
  RouterConfig config;
  config.llm_fallback_enabled = false;
  auto router = MakeRouter(config);

  auto result = router->process("今天天气");

  EXPECT_TRUE(result.failed);
  EXPECT_FALSE(result.from_fast_intent);
  EXPECT_FALSE(result.llm_called);
  EXPECT_EQ(result.failure_reason, "no intent matched");
}

// --- Test 7: FastDisabled ---
TEST(IntentRouterTest, FastDisabled) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto router = MakeRouter(config);

  auto result = router->process("前进");

  EXPECT_FALSE(result.failed);
  EXPECT_FALSE(result.from_fast_intent);
  EXPECT_TRUE(result.llm_called);
  EXPECT_TRUE(result.decision.matched);
  // Default mock → query_introduce
  EXPECT_EQ(result.decision.intent_name, "query");
  EXPECT_EQ(result.decision.action, "introduce");
}

// --- Test 8: LlmReturnsMoveJson ---
// Override mock to return a "move" JSON → mapper produces action/move.
TEST(IntentRouterTest, LlmReturnsMoveJson) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_json_override("move_test",
      R"({"kind":"move","direction":"forward","target":"","reply":"好的，向前走。"})");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("move_test please");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.llm_called);
  EXPECT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "action");
  EXPECT_EQ(result.decision.action, "move");
  EXPECT_EQ(result.decision.target, "chassis");
  EXPECT_EQ(result.decision.value, "forward");
}
TEST(IntentRouterTest, LlmReturnsLiftJsonAsDistinctLiftAction) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_json_override("lift_test",
      R"({"kind":"lift","direction":"up","target":"","reply":"好的，升高。"})");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("lift_test please");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.llm_called);
  EXPECT_TRUE(result.decision.matched);
  EXPECT_EQ(result.route_source, "llm");
  EXPECT_EQ(result.decision.intent_name, "action");
  EXPECT_EQ(result.decision.action, "lift");
  EXPECT_EQ(result.decision.target, "lift");
  EXPECT_EQ(result.decision.value, "up");
}

// --- Test 9: LlmReturnsFindJson ---
TEST(IntentRouterTest, LlmReturnsFindJson) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_json_override("find cup",
      R"({"kind":"find","direction":"","target":"cup","reply":"我来找杯子。"})");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("find cup");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.llm_called);
  EXPECT_EQ(result.decision.intent_name, "action");
  EXPECT_EQ(result.decision.action, "find");
  EXPECT_EQ(result.decision.target, "cup");
}

// --- Test 10: LlmReturnsShutdown (requires confirmation) ---
TEST(IntentRouterTest, LlmReturnsShutdown) {
  RouterConfig config;
  config.allow_fast_intent = false;
  auto mock = std::make_unique<MockLlmIntentClient>();
  mock->set_json_override("shutdown",
      R"({"kind":"system_shutdown","direction":"","target":"","reply":"确认要关机吗？"})");
  IntentRouter router(config, std::move(mock));

  auto result = router.process("shutdown system");

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.decision.matched);
  EXPECT_EQ(result.decision.intent_name, "system");
  EXPECT_EQ(result.decision.action, "shutdown");
  EXPECT_TRUE(result.decision.requires_confirmation);
}

// --- to_intent_lite mapping ---
TEST(IntentRouterTest, ToIntentLite) {
  RouterResult result;
  result.decision.matched = true;
  result.decision.intent_name = "action";
  result.decision.action = "move";
  result.decision.target = "chassis";
  result.decision.location = "";
  result.decision.value = "forward";
  result.decision.confidence = 1.0f;
  result.decision.requires_confirmation = false;

  IntentLiteFields fields;
  IntentRouter::to_intent_lite(result, "t1", "r1", "u1", 100, fields);

  EXPECT_EQ(fields.intent_name, "action");
  EXPECT_EQ(fields.action, "move");
  EXPECT_EQ(fields.target, "chassis");
  EXPECT_EQ(fields.value, "forward");
  EXPECT_FLOAT_EQ(fields.confidence, 1.0f);
  EXPECT_FALSE(fields.requires_confirmation);
}

// --- to_tts_request mapping ---
TEST(IntentRouterTest, ToTtsRequestReturnsTrue) {
  RouterResult result;
  result.decision.matched = true;
  result.decision.tts_reply = "好的，前进。";

  TtsRequestFields fields;
  bool has_tts = IntentRouter::to_tts_request(result, "t1", "r1", 100, fields);

  EXPECT_TRUE(has_tts);
  EXPECT_EQ(fields.text, "好的，前进。");
  EXPECT_EQ(fields.priority, 1);
  EXPECT_EQ(fields.source, "intent_router");
}

TEST(IntentRouterTest, ToTtsRequestReturnsFalseWhenFailed) {
  RouterResult result;
  result.failed = true;

  TtsRequestFields fields;
  bool has_tts = IntentRouter::to_tts_request(result, "t1", "r1", 100, fields);

  EXPECT_FALSE(has_tts);
}

TEST(IntentRouterTest, ToTtsRequestReturnsFalseWhenEmptyReply) {
  RouterResult result;
  result.decision.matched = true;
  result.decision.tts_reply = "";

  TtsRequestFields fields;
  bool has_tts = IntentRouter::to_tts_request(result, "t1", "r1", 100, fields);

  EXPECT_FALSE(has_tts);
}

}  // namespace
}  // namespace k1muse_voice_intent
