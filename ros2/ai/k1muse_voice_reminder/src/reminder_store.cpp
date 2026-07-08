#include "k1muse_voice_reminder/reminder_store.hpp"

#include <sqlite3.h>

#include <utility>

namespace k1muse_voice_reminder {

namespace {
// Safely get text from sqlite3 column, returning empty string for NULL.
std::string SafeColumnText(sqlite3_stmt* stmt, int col) {
  const auto* text = sqlite3_column_text(stmt, col);
  return text != nullptr ? reinterpret_cast<const char*>(text) : "";
}
}  // namespace

ReminderStore::ReminderStore(std::string database_path)
    : database_path_(std::move(database_path)) {}

ReminderStore::~ReminderStore() {
  if (db_ != nullptr) {
    sqlite3_close(db_);
  }
}

bool ReminderStore::Open() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (sqlite3_open(database_path_.c_str(), &db_) != SQLITE_OK) {
    last_error_ = sqlite3_errmsg(db_);
    return false;
  }
  return Exec(
      "CREATE TABLE IF NOT EXISTS reminders ("
      "id TEXT PRIMARY KEY,"
      "trace_id TEXT NOT NULL,"
      "request_id TEXT,"
      "text TEXT NOT NULL,"
      "run_at_iso TEXT NOT NULL,"
      "timezone TEXT NOT NULL,"
      "status TEXT NOT NULL,"
      "created_at_iso TEXT NOT NULL,"
      "fired_at_iso TEXT);"
      "CREATE INDEX IF NOT EXISTS idx_reminders_due ON reminders(status, run_at_iso);");
}

bool ReminderStore::Exec(const std::string& sql) {
  char* err = nullptr;
  const auto rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err);
  if (rc != SQLITE_OK) {
    last_error_ = err != nullptr ? err : sqlite3_errmsg(db_);
    sqlite3_free(err);
    return false;
  }
  return true;
}

bool ReminderStore::Upsert(const ReminderRecord& record) {
  std::lock_guard<std::mutex> lock(mutex_);
  sqlite3_stmt* stmt = nullptr;
  const char* sql =
      "INSERT OR REPLACE INTO reminders "
      "(id, trace_id, request_id, text, run_at_iso, timezone, status, created_at_iso, fired_at_iso) "
      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    last_error_ = sqlite3_errmsg(db_);
    return false;
  }
  sqlite3_bind_text(stmt, 1, record.id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, record.trace_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, record.request_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, record.text.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 5, record.run_at_iso.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 6, record.timezone.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 7, record.status.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 8, record.created_at_iso.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 9, record.fired_at_iso.c_str(), -1, SQLITE_TRANSIENT);
  const auto rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    last_error_ = sqlite3_errmsg(db_);
    return false;
  }
  return true;
}

std::vector<ReminderRecord> ReminderStore::LoadPending() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ReminderRecord> out;
  sqlite3_stmt* stmt = nullptr;
  const char* sql =
      "SELECT id, trace_id, request_id, text, run_at_iso, timezone, status, created_at_iso, "
      "fired_at_iso FROM reminders WHERE status='pending' ORDER BY run_at_iso ASC;";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    last_error_ = sqlite3_errmsg(db_);
    return out;
  }
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    ReminderRecord r;
    r.id = SafeColumnText(stmt, 0);
    r.trace_id = SafeColumnText(stmt, 1);
    r.request_id = SafeColumnText(stmt, 2);
    r.text = SafeColumnText(stmt, 3);
    r.run_at_iso = SafeColumnText(stmt, 4);
    r.timezone = SafeColumnText(stmt, 5);
    r.status = SafeColumnText(stmt, 6);
    r.created_at_iso = SafeColumnText(stmt, 7);
    r.fired_at_iso = SafeColumnText(stmt, 8);
    out.push_back(std::move(r));
  }
  sqlite3_finalize(stmt);
  return out;
}

std::vector<ReminderRecord> ReminderStore::LoadDue(const std::string& now_iso) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ReminderRecord> out;
  sqlite3_stmt* stmt = nullptr;
  const char* sql =
      "SELECT id, trace_id, request_id, text, run_at_iso, timezone, status, created_at_iso, "
      "fired_at_iso FROM reminders WHERE status='pending' AND run_at_iso <= ? "
      "ORDER BY run_at_iso ASC;";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    last_error_ = sqlite3_errmsg(db_);
    return out;
  }
  sqlite3_bind_text(stmt, 1, now_iso.c_str(), -1, SQLITE_TRANSIENT);
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    ReminderRecord r;
    r.id = SafeColumnText(stmt, 0);
    r.trace_id = SafeColumnText(stmt, 1);
    r.request_id = SafeColumnText(stmt, 2);
    r.text = SafeColumnText(stmt, 3);
    r.run_at_iso = SafeColumnText(stmt, 4);
    r.timezone = SafeColumnText(stmt, 5);
    r.status = SafeColumnText(stmt, 6);
    r.created_at_iso = SafeColumnText(stmt, 7);
    r.fired_at_iso = SafeColumnText(stmt, 8);
    out.push_back(std::move(r));
  }
  sqlite3_finalize(stmt);
  return out;
}

bool ReminderStore::MarkCancelled(const std::string& id, const std::string& cancelled_at_iso) {
  std::lock_guard<std::mutex> lock(mutex_);
  sqlite3_stmt* stmt = nullptr;
  const char* sql = "UPDATE reminders SET status='cancelled', fired_at_iso=? WHERE id=? AND status='pending';";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    last_error_ = sqlite3_errmsg(db_);
    return false;
  }
  sqlite3_bind_text(stmt, 1, cancelled_at_iso.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, id.c_str(), -1, SQLITE_TRANSIENT);
  const auto rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    last_error_ = sqlite3_errmsg(db_);
    return false;
  }
  return true;
}

bool ReminderStore::MarkFired(const std::string& id, const std::string& fired_at_iso) {
  std::lock_guard<std::mutex> lock(mutex_);
  sqlite3_stmt* stmt = nullptr;
  const char* sql = "UPDATE reminders SET status='fired', fired_at_iso=? WHERE id=?;";
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    last_error_ = sqlite3_errmsg(db_);
    return false;
  }
  sqlite3_bind_text(stmt, 1, fired_at_iso.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, id.c_str(), -1, SQLITE_TRANSIENT);
  const auto rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  if (rc != SQLITE_DONE) {
    last_error_ = sqlite3_errmsg(db_);
    return false;
  }
  return true;
}

}  // namespace k1muse_voice_reminder
