#pragma once

#include <mutex>
#include <string>
#include <vector>

struct sqlite3;

namespace k1muse_voice_reminder {

struct ReminderRecord {
  std::string id;
  std::string trace_id;
  std::string request_id;
  std::string text;
  std::string run_at_iso;
  std::string timezone = "Asia/Shanghai";
  std::string status = "pending";
  std::string created_at_iso;
  std::string fired_at_iso;
};

class IReminderStore {
 public:
  virtual ~IReminderStore() = default;
  virtual bool Upsert(const ReminderRecord& record) = 0;
  virtual bool MarkCancelled(const std::string& id, const std::string& cancelled_at_iso) = 0;
  virtual std::vector<ReminderRecord> LoadPending() = 0;
};

class ReminderStore final : public IReminderStore {
 public:
  explicit ReminderStore(std::string database_path);
  ~ReminderStore();

  ReminderStore(const ReminderStore&) = delete;
  ReminderStore& operator=(const ReminderStore&) = delete;

  bool Open();
  bool Upsert(const ReminderRecord& record) override;
  bool MarkCancelled(const std::string& id, const std::string& cancelled_at_iso) override;
  std::vector<ReminderRecord> LoadPending() override;
  std::vector<ReminderRecord> LoadDue(const std::string& now_iso);
  bool MarkFired(const std::string& id, const std::string& fired_at_iso);

  const std::string& last_error() const { return last_error_; }

 private:
  bool Exec(const std::string& sql);

  std::string database_path_;
  std::string last_error_;
  sqlite3* db_ = nullptr;
  mutable std::mutex mutex_;  // guards all DB access from timer + intent callbacks
};

}  // namespace k1muse_voice_reminder
