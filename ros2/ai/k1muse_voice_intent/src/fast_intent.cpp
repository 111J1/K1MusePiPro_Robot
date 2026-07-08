#include "k1muse_voice_intent/fast_intent.hpp"

#include <regex>
#include <string>
#include <vector>

namespace k1muse_voice_intent {
namespace {

struct FastRule {
  std::regex pattern;
  std::string intent_name;
  std::string action;
  std::string target;
  std::string value;
  std::string tts_reply;
  bool requires_confirmation;
  FastIntentCategory category;
  SlotState slot_state;
  std::string route_reason;
  float distance_m = 0.0f;
  float angle_rad = 0.0f;
};

const std::regex::flag_type kRegexFlags = std::regex::icase | std::regex::optimize;

std::vector<FastRule> BuildRules() {
  std::vector<FastRule> rules;

  // Safety-critical stop commands. These must not wait for LLM.
  rules.push_back({std::regex("停止|停下|别动|急停|不要动|先别动|停一下|暂停|刹住", kRegexFlags),
                    "action", "stop", "chassis", "", "已停止。", false,
                    FastIntentCategory::kSafetyFast, SlotState::kNoSlotNeeded,
                    "safety_stop"});

  // Simple relative movement. RoutePolicy decides whether long text is safe.
  rules.push_back({std::regex("前进|向前|往前", kRegexFlags),
                    "action", "move", "chassis", "forward", "好的，向前走。", false,
                    FastIntentCategory::kSimpleActionFast, SlotState::kSlotValid,
                    "simple_move_forward", 0.5f, 0.0f});
  rules.push_back({std::regex("后退|往后|倒车", kRegexFlags),
                    "action", "move", "chassis", "backward", "好的，后退。", false,
                    FastIntentCategory::kSimpleActionFast, SlotState::kSlotValid,
                    "simple_move_backward", 0.5f, 0.0f});
  rules.push_back({std::regex("左转|向左|往左", kRegexFlags),
                    "action", "move", "chassis", "left", "好的，左转。", false,
                    FastIntentCategory::kSimpleActionFast, SlotState::kSlotValid,
                    "simple_move_left", 0.5f, 0.0f});
  rules.push_back({std::regex("右转|向右|往右", kRegexFlags),
                    "action", "move", "chassis", "right", "好的，右转。", false,
                    FastIntentCategory::kSimpleActionFast, SlotState::kSlotValid,
                    "simple_move_right", 0.5f, 0.0f});

  rules.push_back({std::regex("旋转|转一圈|转个圈|原地转一圈|转个身", kRegexFlags),
                    "action", "rotate", "chassis", "360", "好的，旋转。", false,
                    FastIntentCategory::kSimpleActionFast, SlotState::kSlotValid,
                    "simple_rotate", 0.0f, 6.2832f});

  // Lift is a distinct action. It must never be encoded as chassis move.
  rules.push_back({std::regex("升高|上升|抬高|往上", kRegexFlags),
                    "action", "lift", "lift", "up", "好的，升高。", false,
                    FastIntentCategory::kSimpleActionFast, SlotState::kSlotValid,
                    "simple_lift_up"});
  rules.push_back({std::regex("降低|下降|往下", kRegexFlags),
                    "action", "lift", "lift", "down", "好的，降低。", false,
                    FastIntentCategory::kSimpleActionFast, SlotState::kSlotValid,
                    "simple_lift_down"});

  rules.push_back({std::regex("你是谁|自我介绍|介绍一下自己|你能做什么|你会什么|你能干嘛|你可以做什么|你有什么功能", kRegexFlags),
                    "query", "introduce", "self", "", "我是小慕，你的智能机器人助手。", false,
                    FastIntentCategory::kQueryFast, SlotState::kNoSlotNeeded,
                    "local_query_introduce"});
  rules.push_back({std::regex("几点|时间|现在什么时候", kRegexFlags),
                    "query", "time", "system", "", "我暂时还不能准确读取当前时间。", false,
                    FastIntentCategory::kQueryFast, SlotState::kNoSlotNeeded,
                    "local_query_time_deferred"});

  rules.push_back({std::regex("你好|您好|早上好|晚上好", kRegexFlags),
                    "chat", "chat", "", "", "你好！有什么可以帮你的？", false,
                    FastIntentCategory::kChatFast, SlotState::kNoSlotNeeded,
                    "local_chat_greeting"});
  rules.push_back({std::regex("谢谢|多谢", kRegexFlags),
                    "chat", "chat", "", "", "不客气！", false,
                    FastIntentCategory::kChatFast, SlotState::kNoSlotNeeded,
                    "local_chat_thanks"});
  rules.push_back({std::regex("再见|拜拜|辛苦了|晚安", kRegexFlags),
                    "chat", "chat", "", "", "好的，再见。", false,
                    FastIntentCategory::kChatFast, SlotState::kNoSlotNeeded,
                    "local_chat_bye"});

  rules.push_back({std::regex("查看提醒|提醒列表|有哪些提醒|我的提醒", kRegexFlags),
                    "reminder", "list", "reminder", "", "", false,
                    FastIntentCategory::kExtractiveFast, SlotState::kNoSlotNeeded,
                    "local_reminder_list"});

  rules.push_back({std::regex("关机|关闭系统|关掉", kRegexFlags),
                    "system", "shutdown", "system", "", "确认要关机吗？", true,
                    FastIntentCategory::kQueryFast, SlotState::kNoSlotNeeded,
                    "local_system_shutdown_confirm"});
  rules.push_back({std::regex("重启|重新启动|重新开机", kRegexFlags),
                    "system", "reboot", "system", "", "确认要重启吗？", true,
                    FastIntentCategory::kQueryFast, SlotState::kNoSlotNeeded,
                    "local_system_reboot_confirm"});

  return rules;
}

const std::vector<FastRule>& GetRules() {
  static const auto rules = BuildRules();
  return rules;
}

std::string TrimSpaces(std::string value) {
  while (!value.empty() && value.front() == ' ') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == ' ') {
    value.pop_back();
  }
  return value;
}

void StripPrefix(std::string* value, const std::vector<std::string>& prefixes) {
  for (const auto& prefix : prefixes) {
    if (value->rfind(prefix, 0) == 0) {
      *value = TrimSpaces(value->substr(prefix.size()));
      return;
    }
  }
}

void StripSuffix(std::string* value, const std::vector<std::string>& suffixes) {
  for (const auto& suffix : suffixes) {
    if (value->size() >= suffix.size() &&
        value->compare(value->size() - suffix.size(), suffix.size(), suffix) == 0) {
      *value = TrimSpaces(value->substr(0, value->size() - suffix.size()));
      return;
    }
  }
}

}  // namespace

const char* ToString(FastIntentCategory category) {
  switch (category) {
    case FastIntentCategory::kSafetyFast: return "safety_fast";
    case FastIntentCategory::kSimpleActionFast: return "simple_action_fast";
    case FastIntentCategory::kQueryFast: return "query_fast";
    case FastIntentCategory::kChatFast: return "chat_fast";
    case FastIntentCategory::kExtractiveFast: return "extractive_fast";
    case FastIntentCategory::kSemanticLlm: return "semantic_llm";
    case FastIntentCategory::kRecoverableUnknown: return "recoverable_unknown";
    default: return "none";
  }
}

const char* ToString(SlotState slot_state) {
  switch (slot_state) {
    case SlotState::kNoSlotNeeded: return "no_slot_needed";
    case SlotState::kSlotValid: return "slot_valid";
    case SlotState::kSlotMissing: return "slot_missing";
    case SlotState::kSlotAmbiguous: return "slot_ambiguous";
    default: return "unknown";
  }
}

FastIntentCandidate MatchFastIntentCandidate(const std::string& text) {
  FastIntentCandidate candidate;
  const auto& rules = GetRules();
  for (const auto& rule : rules) {
    if (std::regex_search(text, rule.pattern)) {
      candidate.decision.matched = true;
      candidate.decision.intent_name = rule.intent_name;
      candidate.decision.action = rule.action;
      candidate.decision.target = rule.target;
      candidate.decision.value = rule.value;
      candidate.decision.confidence = 1.0f;
      candidate.decision.requires_confirmation = rule.requires_confirmation;
      candidate.decision.tts_reply = rule.tts_reply;
      candidate.decision.distance_m = rule.distance_m;
      candidate.decision.angle_rad = rule.angle_rad;
      candidate.category = rule.category;
      candidate.slot_state = rule.slot_state;
      candidate.route_reason = rule.route_reason;
      break;
    }
  }

  if (!candidate.decision.matched) {
    const std::vector<std::string> find_markers = {
      "帮我找", "帮我拿", "帮我取", "寻找", "搜索", "找", "拿下", "取一下", "拿过来", "拿来", "拿"};
    for (const auto& marker : find_markers) {
      const auto pos = text.find(marker);
      if (pos != std::string::npos) {
        auto target = TrimSpaces(text.substr(pos + marker.size()));
        StripPrefix(&target, {"一下", "那个", "一下那个"});
        StripSuffix(&target, {"过来", "一下", "给我", "吧"});
        if (!target.empty()) {
          candidate.decision.matched = true;
          candidate.decision.intent_name = "action";
          candidate.decision.action = "find";
          candidate.decision.target = target;
          candidate.decision.confidence = 0.9f;
          candidate.decision.tts_reply = "好的，我先帮你找" + target + "。";
          candidate.category = FastIntentCategory::kExtractiveFast;
          candidate.slot_state = SlotState::kSlotValid;
          candidate.route_reason = "extractive_find_target";
          break;
        }
      }
    }
  }

  if (!candidate.decision.matched) {
    const std::vector<std::string> reminder_markers = {"提醒我", "设置提醒", "定个闹钟", "记得叫我"};
    for (const auto& marker : reminder_markers) {
      const auto pos = text.find(marker);
      if (pos != std::string::npos) {
        const auto prefix = TrimSpaces(text.substr(0, pos));
        const auto suffix = TrimSpaces(text.substr(pos + marker.size()));
        auto reminder_text = TrimSpaces(prefix + suffix);
        if (!reminder_text.empty()) {
          candidate.decision.matched = true;
          candidate.decision.intent_name = "reminder";
          candidate.decision.action = "create";
          candidate.decision.target = reminder_text;
          candidate.decision.value = reminder_text;
          candidate.decision.confidence = 0.85f;
          candidate.decision.tts_reply = "好的，我会提醒你。";
          candidate.category = FastIntentCategory::kExtractiveFast;
          candidate.slot_state = SlotState::kSlotValid;
          candidate.route_reason = "extractive_reminder_create";
          break;
        }
        candidate.category = FastIntentCategory::kExtractiveFast;
        candidate.slot_state = SlotState::kSlotMissing;
        candidate.route_reason = "reminder_text_missing";
      }
    }
  }

  return candidate;
}

IntentDecision MatchFastIntent(const std::string& text) {
  return MatchFastIntentCandidate(text).decision;
}

}  // namespace k1muse_voice_intent
