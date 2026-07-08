#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_voice_reminder/reminder_handler.hpp"

namespace {

// A mock store that keeps records in memory.
class MockReminderStore : public k1muse_voice_reminder::IReminderStore {
 public:
  bool Upsert(const k1muse_voice_reminder::ReminderRecord& record) override {
    records_[record.id] = record;
    return true;
  }

  bool MarkCancelled(const std::string& id, const std::string& cancelled_at_iso) override {
    auto it = records_.find(id);
    if (it == records_.end()) return false;
    it->second.status = "cancelled";
    it->second.fired_at_iso = cancelled_at_iso;
    return true;
  }

  std::vector<k1muse_voice_reminder::ReminderRecord> LoadPending() override {
    std::vector<k1muse_voice_reminder::ReminderRecord> out;
    for (const auto& [id, r] : records_) {
      if (r.status == "pending") out.push_back(r);
    }
    return out;
  }

 private:
  std::unordered_map<std::string, k1muse_voice_reminder::ReminderRecord> records_;
};

// A mock TTS publisher that records published messages.
class MockTtsPublisher : public k1muse_voice_reminder::ITtsTextPublisher {
 public:
  void Publish(const k1muse_voice_msgs::msg::TtsTextRequest& request) override {
    published_.push_back(request);
  }

  const std::vector<k1muse_voice_msgs::msg::TtsTextRequest>& published() const { return published_; }

 private:
  std::vector<k1muse_voice_msgs::msg::TtsTextRequest> published_;
};

k1muse_voice_msgs::msg::IntentLite MakeIntent(
    const std::string& intent_name,
    const std::string& action,
    const std::string& target,
    const std::string& value) {
  k1muse_voice_msgs::msg::IntentLite msg;
  msg.trace_id = "trace-1";
  msg.request_id = "req-1";
  msg.epoch = 42;
  msg.intent_name = intent_name;
  msg.action = action;
  msg.target = target;
  msg.value = value;
  return msg;
}

}  // namespace

TEST(ReminderHandler, CreateReminderStoresRecord) {
  MockReminderStore store;
  k1muse_voice_reminder::ReminderIntentHandler handler(&store, "Asia/Shanghai");

  const auto intent = MakeIntent("reminder", "create", "喝水", "PT5M");
  const auto result = handler.HandleIntent(intent);

  EXPECT_TRUE(result.created);
  EXPECT_TRUE(result.error.empty());

  const auto pending = store.LoadPending();
  ASSERT_EQ(pending.size(), 1u);
  EXPECT_EQ(pending.front().text, "喝水");
  EXPECT_EQ(pending.front().request_id, "req-1");
}

TEST(ReminderHandler, CancelReminderRemovesFromStore) {
  MockReminderStore store;
  k1muse_voice_reminder::ReminderIntentHandler handler(&store, "Asia/Shanghai");

  // Create first
  handler.HandleIntent(MakeIntent("reminder", "create", "喝水", "PT5M"));
  ASSERT_EQ(store.LoadPending().size(), 1u);

  // Cancel
  const auto result = handler.HandleIntent(MakeIntent("reminder", "cancel", "喝水", ""));
  EXPECT_TRUE(result.cancelled);
  EXPECT_FALSE(result.tts_reply.empty());
  EXPECT_EQ(store.LoadPending().size(), 0u);
}

TEST(ReminderHandler, ListReminderReportsCount) {
  MockReminderStore store;
  k1muse_voice_reminder::ReminderIntentHandler handler(&store, "Asia/Shanghai");

  handler.HandleIntent(MakeIntent("reminder", "create", "喝水", "PT5M"));
  handler.HandleIntent(MakeIntent("reminder", "create", "吃药", "PT10M"));

  const auto result = handler.HandleIntent(MakeIntent("reminder", "list", "", ""));
  EXPECT_TRUE(result.listed);
  EXPECT_NE(result.tts_reply.find("2"), std::string::npos);
}

TEST(ReminderHandler, NonReminderIntentIsIgnored) {
  MockReminderStore store;
  k1muse_voice_reminder::ReminderIntentHandler handler(&store, "Asia/Shanghai");

  const auto intent = MakeIntent("weather", "query", "北京", "");
  const auto result = handler.HandleIntent(intent);

  EXPECT_TRUE(result.ignored);
  EXPECT_FALSE(result.created);
  EXPECT_FALSE(result.cancelled);
  EXPECT_FALSE(result.listed);
}

TEST(ReminderHandler, CancelNonexistentReminderReportsNotFound) {
  MockReminderStore store;
  k1muse_voice_reminder::ReminderIntentHandler handler(&store, "Asia/Shanghai");

  const auto result = handler.HandleIntent(MakeIntent("reminder", "cancel", "不存在", ""));
  EXPECT_FALSE(result.cancelled);
  EXPECT_NE(result.tts_reply.find("未找到"), std::string::npos);
}

TEST(ReminderDueHandler, PublishesTtsWithCorrectFields) {
  MockTtsPublisher publisher;
  k1muse_voice_reminder::ReminderDueHandler handler(&publisher);

  handler.OnReminderDue("trace-1", "req-1", 42, "提醒您：喝水");

  ASSERT_EQ(publisher.published().size(), 1u);
  const auto& msg = publisher.published().front();
  EXPECT_EQ(msg.trace_id, "trace-1");
  EXPECT_EQ(msg.request_id, "req-1");
  EXPECT_EQ(msg.epoch, 42u);
  EXPECT_EQ(msg.text, "提醒您：喝水");
  EXPECT_EQ(msg.source, "reminder_node");
  EXPECT_EQ(msg.priority, k1muse_voice_msgs::msg::TtsTextRequest::PRIORITY_REMINDER);
}
