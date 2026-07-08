#include "k1muse_voice_intent/llm_intent_mapper.hpp"

#include <string>

namespace k1muse_voice_intent {
namespace {

std::string DefaultReplyForKind(const std::string& kind,
                                const std::string& direction,
                                const std::string& target) {
  if (kind == "move") {
    if (direction == "backward") return "好的，后退。";
    if (direction == "left") return "好的，左转。";
    if (direction == "right") return "好的，右转。";
    return "好的，向前走。";
  }
  if (kind == "stop") return "已停止。";
  if (kind == "rotate") return "好的，我转一圈。";
  if (kind == "lift") {
    return direction == "down" ? "好的，降低。" : "好的，升高。";
  }
  if (kind == "find") {
    return target.empty() ? "好的，我来找。" : "好的，我来找" + target + "。";
  }
  if (kind == "query_introduce") return "我是小慕，你的智能机器人助手。";
  if (kind == "query_time") return "让我看看时间。";
  if (kind == "reminder_create") return "好的，我会提醒你。";
  if (kind == "reminder_list") return "好的，查看提醒列表。";
  if (kind == "system_shutdown") return "确认要关机吗？";
  if (kind == "system_reboot") return "确认要重启吗？";
  if (kind == "chat") return "你好！有什么可以帮你的？";
  if (kind == "ask_repeat") return "我没有听清，请再说一遍。";
  return "我还没理解，请换一种说法。";
}

}  // namespace

IntentDecision MapLlmOutput(const LlmValidationResult& validated) {
  if (!validated.valid) {
    IntentDecision d;
    d.matched = false;
    return d;
  }

  IntentDecision d;
  const std::string& kind = validated.kind;

  if (kind == "move") {
    if (validated.direction != "forward" &&
        validated.direction != "backward" &&
        validated.direction != "left" &&
        validated.direction != "right") {
      d.matched = false;
      return d;
    }
    d.matched = true;
    d.intent_name = "action";
    d.action = "move";
    d.target = "chassis";
    d.value = validated.direction;
    d.distance_m = 0.5f;
    d.confidence = 1.0f;

  } else if (kind == "stop") {
    d.matched = true;
    d.intent_name = "action";
    d.action = "stop";
    d.target = "chassis";
    d.confidence = 1.0f;

  } else if (kind == "rotate") {
    d.matched = true;
    d.intent_name = "action";
    d.action = "rotate";
    d.target = "chassis";
    d.value = "360";
    d.angle_rad = 6.2832f;
    d.confidence = 1.0f;

  } else if (kind == "lift") {
    if (validated.direction != "up" && validated.direction != "down") {
      d.matched = false;
      return d;
    }
    d.matched = true;
    d.intent_name = "action";
    d.action = "lift";
    d.target = "lift";
    d.value = validated.direction;
    d.confidence = 1.0f;

  } else if (kind == "find") {
    if (validated.target.empty()) {
      d.matched = false;
      return d;
    }
    d.matched = true;
    d.intent_name = "action";
    d.action = "find";
    d.target = validated.target;
    d.confidence = 1.0f;

  } else if (kind == "query_introduce") {
    d.matched = true;
    d.intent_name = "query";
    d.action = "introduce";
    d.target = "self";
    d.confidence = 1.0f;

  } else if (kind == "query_time") {
    d.matched = true;
    d.intent_name = "query";
    d.action = "time";
    d.target = "system";
    d.confidence = 1.0f;

  } else if (kind == "reminder_create") {
    if (validated.target.empty()) {
      d.matched = false;
      return d;
    }
    d.matched = true;
    d.intent_name = "reminder";
    d.action = "create";
    d.target = validated.target;
    d.value = validated.target;
    d.confidence = 1.0f;

  } else if (kind == "reminder_list") {
    d.matched = true;
    d.intent_name = "reminder";
    d.action = "list";
    d.target = "reminder";
    d.confidence = 1.0f;

  } else if (kind == "system_shutdown") {
    d.matched = true;
    d.intent_name = "system";
    d.action = "shutdown";
    d.target = "system";
    d.requires_confirmation = true;
    d.confidence = 1.0f;

  } else if (kind == "system_reboot") {
    d.matched = true;
    d.intent_name = "system";
    d.action = "reboot";
    d.target = "system";
    d.requires_confirmation = true;
    d.confidence = 1.0f;

  } else if (kind == "chat") {
    d.matched = true;
    d.intent_name = "chat";
    d.action = "chat";
    d.target = "";
    d.confidence = 1.0f;

  } else if (kind == "ask_repeat" || kind == "unknown") {
    d.matched = true;
    d.intent_name = "unknown";
    d.action = "ask_repeat";
    d.confidence = 0.3f;

  } else {
    d.matched = false;
    return d;
  }

  // Prefer LLM reply, but keep intent usable when a small local model
  // classifies correctly and leaves reply empty.
  d.tts_reply = validated.reply;
  if (d.tts_reply.empty()) {
    d.tts_reply = DefaultReplyForKind(kind, validated.direction, validated.target);
  }

  return d;
}

}  // namespace k1muse_voice_intent
