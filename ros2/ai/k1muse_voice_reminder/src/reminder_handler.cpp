#include "k1muse_voice_reminder/reminder_handler.hpp"

#include <algorithm>
#include <utility>

namespace k1muse_voice_reminder {

ReminderIntentHandler::ReminderIntentHandler(IReminderStore* store, std::string timezone)
    : store_(store), parser_(std::move(timezone)) {}

ReminderIntentResult ReminderIntentHandler::HandleIntent(
    const k1muse_voice_msgs::msg::IntentLite& intent) {
  ReminderIntentResult out;
  if (intent.intent_name != "reminder") {
    out.ignored = true;
    return out;
  }

  if (intent.action == "create") {
    const auto parsed = parser_.ParseCreateIntent(intent.trace_id, intent.target, intent.value);
    if (!parsed.ok) {
      out.error = parsed.error;
      return out;
    }

    ReminderRecord record;
    record.id = MakeReminderId(intent.trace_id, intent.target);
    record.trace_id = intent.trace_id;
    record.request_id = intent.request_id;
    record.text = parsed.text;
    record.run_at_iso = parsed.run_at_iso;
    record.timezone = parsed.timezone;
    record.status = "pending";
    record.created_at_iso = FormatShanghaiIsoFromNow(0);
    if (!store_->Upsert(record)) {
      out.error = "failed to store reminder";
      return out;
    }
    out.created = true;
  } else if (intent.action == "cancel") {
    auto pending = store_->LoadPending();
    bool found = false;
    for (const auto& r : pending) {
      if (r.id == intent.target || r.text.find(intent.target) != std::string::npos) {
        store_->MarkCancelled(r.id, FormatShanghaiIsoFromNow(0));
        found = true;
        out.tts_reply = "已取消提醒：" + r.text;
        break;
      }
    }
    if (!found) {
      out.tts_reply = "未找到该提醒。";
    }
    out.cancelled = found;
  } else if (intent.action == "list") {
    auto pending = store_->LoadPending();
    if (pending.empty()) {
      out.tts_reply = "当前没有待办提醒。";
    } else {
      out.tts_reply = "您有 " + std::to_string(pending.size()) + " 个提醒：";
      for (size_t i = 0; i < std::min(pending.size(), static_cast<size_t>(3)); ++i) {
        out.tts_reply += pending[i].text;
        if (i < 2 && i < pending.size() - 1) out.tts_reply += "、";
      }
      out.tts_reply += "。";
    }
    out.listed = true;
  } else {
    out.ignored = true;
  }
  return out;
}

ReminderDueHandler::ReminderDueHandler(ITtsTextPublisher* publisher)
    : publisher_(publisher) {}

void ReminderDueHandler::OnReminderDue(
    const std::string& trace_id,
    const std::string& request_id,
    uint64_t epoch,
    const std::string& speech_text) {
  k1muse_voice_msgs::msg::TtsTextRequest request;
  request.trace_id = trace_id;
  request.request_id = request_id;
  request.epoch = epoch;
  request.text = speech_text;
  request.source = "reminder_node";
  request.priority = k1muse_voice_msgs::msg::TtsTextRequest::PRIORITY_REMINDER;
  publisher_->Publish(request);
}

}  // namespace k1muse_voice_reminder
