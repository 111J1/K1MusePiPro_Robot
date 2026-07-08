#include <filesystem>

#include <gtest/gtest.h>

#include "k1muse_voice_reminder/reminder_store.hpp"

TEST(ReminderStore, ReloadsPendingReminderFromSqlite) {
  const auto db_path =
      (std::filesystem::temp_directory_path() / "k1muse_reminder_store_test.sqlite3").string();
  std::filesystem::remove(db_path);

  k1muse_voice_reminder::ReminderStore store(db_path);
  ASSERT_TRUE(store.Open()) << store.last_error();

  k1muse_voice_reminder::ReminderRecord record;
  record.id = "reminder-1";
  record.trace_id = "trace-1";
  record.request_id = "req-1";
  record.text = "drink water";
  record.run_at_iso = "2026-06-15T20:00:00+08:00";
  record.timezone = "Asia/Shanghai";
  record.status = "pending";
  record.created_at_iso = "2026-06-15T19:00:00+08:00";
  ASSERT_TRUE(store.Upsert(record)) << store.last_error();

  k1muse_voice_reminder::ReminderStore reloaded(db_path);
  ASSERT_TRUE(reloaded.Open()) << reloaded.last_error();
  const auto pending = reloaded.LoadPending();
  ASSERT_EQ(pending.size(), 1u);
  EXPECT_EQ(pending.front().text, "drink water");
  EXPECT_EQ(pending.front().request_id, "req-1");

  std::filesystem::remove(db_path);
}

TEST(ReminderStore, MarkCancelledRemovesFromPending) {
  const auto db_path =
      (std::filesystem::temp_directory_path() / "k1muse_reminder_cancel_test.sqlite3").string();
  std::filesystem::remove(db_path);

  k1muse_voice_reminder::ReminderStore store(db_path);
  ASSERT_TRUE(store.Open()) << store.last_error();

  k1muse_voice_reminder::ReminderRecord record;
  record.id = "reminder-cancel-1";
  record.trace_id = "trace-1";
  record.request_id = "req-1";
  record.text = "to be cancelled";
  record.run_at_iso = "2026-06-15T20:00:00+08:00";
  record.timezone = "Asia/Shanghai";
  record.status = "pending";
  record.created_at_iso = "2026-06-15T19:00:00+08:00";
  ASSERT_TRUE(store.Upsert(record)) << store.last_error();

  // Verify it's pending
  auto pending = store.LoadPending();
  ASSERT_EQ(pending.size(), 1u);

  // Cancel it
  ASSERT_TRUE(store.MarkCancelled("reminder-cancel-1", "2026-06-15T19:30:00+08:00"))
      << store.last_error();

  // Verify it's no longer pending
  pending = store.LoadPending();
  EXPECT_EQ(pending.size(), 0u);

  std::filesystem::remove(db_path);
}

TEST(ReminderStore, LoadDueReturnsOnlyDueReminders) {
  const auto db_path =
      (std::filesystem::temp_directory_path() / "k1muse_reminder_due_test.sqlite3").string();
  std::filesystem::remove(db_path);

  k1muse_voice_reminder::ReminderStore store(db_path);
  ASSERT_TRUE(store.Open()) << store.last_error();

  // Past reminder (should be due)
  k1muse_voice_reminder::ReminderRecord past;
  past.id = "reminder-past";
  past.trace_id = "trace-1";
  past.request_id = "req-1";
  past.text = "past reminder";
  past.run_at_iso = "2026-01-01T00:00:00+08:00";
  past.timezone = "Asia/Shanghai";
  past.status = "pending";
  past.created_at_iso = "2025-12-31T23:00:00+08:00";
  ASSERT_TRUE(store.Upsert(past)) << store.last_error();

  // Future reminder (should NOT be due)
  k1muse_voice_reminder::ReminderRecord future;
  future.id = "reminder-future";
  future.trace_id = "trace-2";
  future.request_id = "req-2";
  future.text = "future reminder";
  future.run_at_iso = "2099-12-31T23:59:59+08:00";
  future.timezone = "Asia/Shanghai";
  future.status = "pending";
  future.created_at_iso = "2026-06-15T19:00:00+08:00";
  ASSERT_TRUE(store.Upsert(future)) << store.last_error();

  // Query with a time between past and future
  const auto due = store.LoadDue("2026-06-15T20:00:00+08:00");
  ASSERT_EQ(due.size(), 1u);
  EXPECT_EQ(due.front().id, "reminder-past");

  std::filesystem::remove(db_path);
}

TEST(ReminderStore, MarkFiredChangesStatus) {
  const auto db_path =
      (std::filesystem::temp_directory_path() / "k1muse_reminder_fired_test.sqlite3").string();
  std::filesystem::remove(db_path);

  k1muse_voice_reminder::ReminderStore store(db_path);
  ASSERT_TRUE(store.Open()) << store.last_error();

  k1muse_voice_reminder::ReminderRecord record;
  record.id = "reminder-fired";
  record.trace_id = "trace-1";
  record.request_id = "req-1";
  record.text = "to be fired";
  record.run_at_iso = "2026-01-01T00:00:00+08:00";
  record.timezone = "Asia/Shanghai";
  record.status = "pending";
  record.created_at_iso = "2025-12-31T23:00:00+08:00";
  ASSERT_TRUE(store.Upsert(record)) << store.last_error();

  ASSERT_TRUE(store.MarkFired("reminder-fired", "2026-01-01T00:00:01+08:00"))
      << store.last_error();

  // After firing, it should no longer appear in LoadDue
  const auto due = store.LoadDue("2099-12-31T23:59:59+08:00");
  EXPECT_EQ(due.size(), 0u);

  std::filesystem::remove(db_path);
}
