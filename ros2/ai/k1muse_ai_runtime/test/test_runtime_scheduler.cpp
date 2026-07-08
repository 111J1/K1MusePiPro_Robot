#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "k1muse_ai_runtime/bounded_cpu_worker.hpp"
#include "k1muse_ai_runtime/runtime_scheduler.hpp"

using namespace std::chrono_literals;
using k1muse_ai_runtime::BoundedCpuWorker;
using k1muse_ai_runtime::CancellationState;
using k1muse_ai_runtime::CancellationToken;
using k1muse_ai_runtime::CommitGenerationGate;
using k1muse_ai_runtime::InferenceJob;
using k1muse_ai_runtime::JobModule;
using k1muse_ai_runtime::JobPriority;
using k1muse_ai_runtime::ProviderClass;
using k1muse_ai_runtime::ResourceGuard;
using k1muse_ai_runtime::RuntimeScheduler;

namespace
{
template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

InferenceJob guarded_job(
  std::string id, JobModule module, std::vector<std::string> * commits = nullptr)
{
  InferenceJob value;
  value.id = std::move(id);
  value.module = module;
  value.provider_class = k1muse_ai_runtime::canonical_provider(module);
  value.priority = k1muse_ai_runtime::canonical_priority(module);
  value.epoch = 7;
  value.deadline = std::chrono::steady_clock::now() + 1s;
  value.epoch_is_current = [](uint64_t epoch) {return epoch == 7;};
  value.module_enabled = [](JobModule) {return true;};
  value.cancel_predicate = []() {return false;};
  value.execute = [](const CancellationToken &) {};
  value.commit_if_current = [commits, id = value.id](const CancellationToken & token) {
      if (!token.try_begin_commit()) {
        return false;
      }
      if (commits) {
        commits->push_back(id);
      }
      return true;
    };
  return value;
}

InferenceJob cpu_job(JobModule module, std::atomic<int> * commits = nullptr)
{
  auto value = guarded_job("cpu", module);
  value.provider_class = ProviderClass::Cpu;
  value.priority = JobPriority::Cpu;
  value.commit_if_current = [commits](const CancellationToken & token) {
      if (!token.try_begin_commit()) {
        return false;
      }
      if (commits) {
        ++(*commits);
      }
      return true;
    };
  return value;
}
}  // namespace

TEST(RuntimeScheduler, PriorityAndSamePriorityFifo)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 8, 100ms);
  std::vector<std::string> order;
  EXPECT_TRUE(scheduler.submit(guarded_job("vision", JobModule::VISION_EP, &order)));
  EXPECT_TRUE(scheduler.submit(guarded_job("tts-1", JobModule::TTS_EP, &order)));
  EXPECT_TRUE(scheduler.submit(guarded_job("asr", JobModule::ASR_EP, &order)));
  EXPECT_TRUE(scheduler.submit(guarded_job("tts-2", JobModule::TTS_EP, &order)));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().committed == 4U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(order, (std::vector<std::string>{"asr", "tts-1", "tts-2", "vision"}));
}

TEST(RuntimeScheduler, SamePriorityFifo)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 4, 100ms);
  std::vector<std::string> order;
  scheduler.submit(guarded_job("one", JobModule::TTS_EP, &order));
  scheduler.submit(guarded_job("two", JobModule::TTS_EP, &order));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().committed == 2U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(order, (std::vector<std::string>{"one", "two"}));
}

TEST(RuntimeScheduler, QueueBound)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 1, 100ms);
  EXPECT_TRUE(scheduler.submit(guarded_job("one", JobModule::ASR_EP)));
  EXPECT_FALSE(scheduler.submit(guarded_job("two", JobModule::ASR_EP)));
  EXPECT_EQ(scheduler.stats().rejected_full, 1U);
}

TEST(RuntimeScheduler, VisionCannotMasqueradeAsAsr)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 100ms);
  auto forged = guarded_job("forged", JobModule::VISION_EP);
  forged.priority = JobPriority::Asr;
  EXPECT_FALSE(scheduler.submit(std::move(forged)));
  EXPECT_EQ(scheduler.stats().rejected_metadata, 1U);
}

TEST(RuntimeScheduler, CpuModuleRejectedByGuardedScheduler)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 100ms);
  auto cpu = cpu_job(JobModule::ASR_CPU);
  EXPECT_FALSE(scheduler.submit(std::move(cpu)));
  EXPECT_EQ(scheduler.stats().rejected_metadata, 1U);
}

TEST(RuntimeScheduler, DeadlineEpochModuleDisabledAndCancelledDrop)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 8, 100ms);
  auto expired = guarded_job("Deadline", JobModule::ASR_EP);
  expired.deadline = std::chrono::steady_clock::now() - 1ms;
  auto stale = guarded_job("Epoch", JobModule::ASR_EP);
  stale.epoch_is_current = [](uint64_t) {return false;};
  auto disabled = guarded_job("ModuleDisabled", JobModule::VISION_EP);
  disabled.module_enabled = [](JobModule) {return false;};
  auto cancelled = guarded_job("Cancelled", JobModule::TTS_EP);
  cancelled.cancel_predicate = []() {return true;};
  scheduler.submit(std::move(expired));
  scheduler.submit(std::move(stale));
  scheduler.submit(std::move(disabled));
  scheduler.submit(std::move(cancelled));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().dropped == 4U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(scheduler.stats().dropped, 4U);
}

TEST(RuntimeScheduler, MissingPredicatesFailClosed)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 1, 100ms);
  InferenceJob incomplete;
  incomplete.id = "incomplete";
  incomplete.module = JobModule::ASR_EP;
  incomplete.provider_class = ProviderClass::GuardedEp;
  incomplete.priority = JobPriority::Asr;
  incomplete.deadline = std::chrono::steady_clock::now() + 1s;
  incomplete.execute = [](const CancellationToken &) {};
  incomplete.commit_if_current = [](const CancellationToken & token) {
      return token.try_begin_commit();
    };
  EXPECT_FALSE(scheduler.submit(std::move(incomplete)));
}

TEST(RuntimeScheduler, RunningEpochChangeSuppressesCommit)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 200ms);
  std::atomic<bool> current{true};
  std::atomic<int> committed{0};
  std::mutex mutex;
  std::condition_variable condition;
  bool executing = false;
  bool release = false;
  auto value = guarded_job("epoch-change", JobModule::ASR_EP);
  value.epoch_is_current = [&](uint64_t) {return current.load();};
  value.execute = [&](const CancellationToken & token) {
      std::unique_lock<std::mutex> lock(mutex);
      executing = true;
      condition.notify_all();
      while (!release && !token.stop_requested()) {
        condition.wait_for(lock, 1ms);
      }
    };
  value.commit_if_current = [&](const CancellationToken & token) {
      if (!current.load()) {
        return false;
      }
      if (!token.try_begin_commit()) {
        return false;
      }
      ++committed;
      return true;
    };
  scheduler.submit(std::move(value));
  scheduler.start();
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 500ms, [&]() {return executing;}));
    current.store(false);
    release = true;
  }
  condition.notify_all();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().executed == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(committed.load(), 0);
}

TEST(RuntimeScheduler, RunningCancelSuppressesCommit)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 200ms);
  std::atomic<bool> cancelled{false};
  std::atomic<int> committed{0};
  auto value = guarded_job("cancel-change", JobModule::TTS_EP);
  value.cancel_predicate = [&]() {return cancelled.load();};
  value.execute = [&](const CancellationToken &) {cancelled.store(true);};
  value.commit_if_current = [&](const CancellationToken & token) {
      if (cancelled.load()) {
        return false;
      }
      if (!token.try_begin_commit()) {
        return false;
      }
      ++committed;
      return true;
    };
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().executed == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(committed.load(), 0);
}

TEST(RuntimeScheduler, GuardIsRecheckedAfterAcquire)
{
  ResourceGuard guard;
  std::optional<ResourceGuard::Lease> held;
  held.emplace(guard.acquire("test-holder"));
  RuntimeScheduler scheduler(guard, 2, 200ms);
  std::atomic<bool> enabled{true};
  std::atomic<int> executed{0};
  auto value = guarded_job("recheck", JobModule::VISION_EP);
  value.module_enabled = [&](JobModule) {return enabled.load();};
  value.execute = [&](const CancellationToken &) {++executed;};
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return !scheduler.stats().active_job.empty();}));
  enabled.store(false);
  held.reset();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().dropped == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(executed.load(), 0);
}

TEST(RuntimeScheduler, ExceptionDoesNotStopFollowingJob)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 4, 100ms);
  std::vector<std::string> order;
  auto failing = guarded_job("Exception", JobModule::ASR_EP, &order);
  failing.execute = [](const CancellationToken &) {throw std::runtime_error("expected");};
  scheduler.submit(std::move(failing));
  scheduler.submit(guarded_job("next", JobModule::VISION_EP, &order));
  scheduler.start();
  ASSERT_TRUE(wait_until(
    [&]() {return scheduler.stats().failed == 1U && scheduler.stats().committed == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(order, (std::vector<std::string>{"next"}));
  EXPECT_EQ(scheduler.stats().failed, 1U);
}

TEST(RuntimeScheduler, MaximumGuardConcurrencyIsOne)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 32, 500ms);
  std::atomic<int> active{0};
  std::atomic<int> maximum{0};
  for (int index = 0; index < 12; ++index) {
    auto value = guarded_job("job-" + std::to_string(index), JobModule::VISION_EP);
    value.execute = [&](const CancellationToken &) {
        const int current = ++active;
        maximum.store(std::max(maximum.load(), current));
        std::this_thread::sleep_for(1ms);
        --active;
      };
    scheduler.submit(std::move(value));
  }
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().executed == 12U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(maximum.load(), 1);
}

TEST(RuntimeScheduler, VisionDoesNotPreempt)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 4, 500ms);
  std::mutex mutex;
  std::condition_variable condition;
  bool vision_started = false;
  bool release_vision = false;
  std::vector<std::string> order;
  auto vision = guarded_job("vision", JobModule::VISION_EP);
  vision.execute = [&](const CancellationToken & token) {
      std::unique_lock<std::mutex> lock(mutex);
      order.push_back("vision-start");
      vision_started = true;
      condition.notify_all();
      while (!release_vision && !token.stop_requested()) {
        condition.wait_for(lock, 1ms);
      }
      order.push_back("vision-end");
    };
  scheduler.submit(std::move(vision));
  scheduler.start();
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 500ms, [&]() {return vision_started;}));
  }
  auto asr = guarded_job("asr", JobModule::ASR_EP);
  asr.commit_if_current = [&](const CancellationToken & token) {
      if (!token.try_begin_commit()) {
        return false;
      }
      order.push_back("asr");
      return true;
    };
  scheduler.submit(std::move(asr));
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_vision = true;
  }
  condition.notify_all();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().committed == 2U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(order, (std::vector<std::string>{"vision-start", "vision-end", "asr"}));
}

TEST(RuntimeScheduler, CooperativeBlockingJobStopsWithinBoundedStop)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 100ms);
  std::atomic<bool> exited{false};
  auto value = guarded_job("cooperative", JobModule::ASR_EP);
  value.execute = [&](const CancellationToken & token) {
      exited.store(false);
      while (!token.stop_requested()) {
        std::this_thread::sleep_for(1ms);
      }
      exited.store(true);
    };
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return !scheduler.stats().active_job.empty();}));
  const auto before = std::chrono::steady_clock::now();
  EXPECT_TRUE(scheduler.stop());
  EXPECT_LT(std::chrono::steady_clock::now() - before, 150ms);
  EXPECT_TRUE(exited.load());
}

TEST(RuntimeScheduler, StopTimeoutReportsFailureAndCanBeRetried)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 0ms);
  std::atomic<bool> exited{false};
  auto value = guarded_job("cooperative-cleanup", JobModule::ASR_EP);
  value.execute = [&](const CancellationToken & token) {
      while (!token.stop_requested()) {
        std::this_thread::sleep_for(1ms);
      }
      std::this_thread::sleep_for(5ms);
      exited.store(true);
    };
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return !scheduler.stats().active_job.empty();}));
  const auto before = std::chrono::steady_clock::now();
  EXPECT_FALSE(scheduler.stop());
  EXPECT_LT(std::chrono::steady_clock::now() - before, 20ms);
  EXPECT_TRUE(scheduler.has_live_thread());
  std::this_thread::sleep_for(15ms);
  EXPECT_TRUE(scheduler.stop());
  EXPECT_FALSE(scheduler.has_live_thread());
  EXPECT_TRUE(exited.load());
}

TEST(RuntimeScheduler, StopTimeoutRetainsJobOwnershipNoUseAfterFree)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 0ms);
  auto owned = std::make_shared<std::atomic<bool>>(false);
  std::weak_ptr<std::atomic<bool>> weak = owned;
  auto value = guarded_job("owned-state", JobModule::ASR_EP);
  value.execute = [owned](const CancellationToken & token) {
      while (!token.stop_requested()) {
        std::this_thread::sleep_for(1ms);
      }
      std::this_thread::sleep_for(5ms);
      owned->store(true);
    };
  owned.reset();
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return !scheduler.stats().active_job.empty();}));
  EXPECT_FALSE(scheduler.stop());
  EXPECT_FALSE(weak.expired());
  std::this_thread::sleep_for(15ms);
  EXPECT_TRUE(scheduler.stop());
  EXPECT_TRUE(weak.expired());
}

TEST(RuntimeScheduler, StateCallbackObservesActiveJobAndQueueChanges)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 100ms);
  std::atomic<int> callbacks{0};
  std::atomic<bool> observed_active{false};
  scheduler.set_state_callback([&]() {
      ++callbacks;
      const auto stats = scheduler.stats();
      if (!stats.active_job.empty()) {
        observed_active.store(true);
      }
    });
  auto value = guarded_job("observable", JobModule::VISION_EP);
  value.execute = [](const CancellationToken &) {std::this_thread::sleep_for(5ms);};
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().committed == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_TRUE(observed_active.load());
  EXPECT_GT(callbacks.load(), 2);
}

TEST(RuntimeScheduler, AtomicCommitGateRejectsEpochChangedAtFinalGate)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 100ms);
  std::mutex generation_mutex;
  uint64_t generation = 1;
  std::atomic<int> committed{0};
  auto value = guarded_job("atomic-gate", JobModule::ASR_EP);
  value.epoch = 1;
  value.epoch_is_current = [&](uint64_t epoch) {
      std::lock_guard<std::mutex> lock(generation_mutex);
      return generation == epoch;
    };
  value.execute = [&](const CancellationToken &) {
      std::lock_guard<std::mutex> lock(generation_mutex);
      generation = 2;
    };
  value.commit_if_current = [&](const CancellationToken & token) {
      std::lock_guard<std::mutex> lock(generation_mutex);
      if (generation != 1) {
        return false;
      }
      if (!token.try_begin_commit()) {
        return false;
      }
      ++committed;
      return true;
    };
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().executed == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(committed.load(), 0);
  EXPECT_EQ(scheduler.stats().committed, 0U);
}

TEST(RuntimeScheduler, GuardTryAcquireSucceedsDuringCommit)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 100ms);
  std::atomic<bool> guard_was_free{false};
  auto value = guarded_job("guard-scope", JobModule::VISION_EP);
  value.commit_if_current = [&](const CancellationToken & token) {
      if (!token.try_begin_commit()) {
        return false;
      }
      auto lease = guard.try_acquire("commit-probe");
      if (!lease) {
        return false;
      }
      guard_was_free.store(true);
      return true;
    };
  scheduler.submit(std::move(value));
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().committed == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_TRUE(guard_was_free.load());
  EXPECT_GE(guard.stats().acquire_count, 2U);
}

TEST(RuntimeScheduler, StaleDropNotifiesStateCallback)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 2, 100ms);
  std::atomic<int> callbacks{0};
  scheduler.set_state_callback([&]() {++callbacks;});
  auto stale = guarded_job("stale-notify", JobModule::TTS_EP);
  stale.epoch_is_current = [](uint64_t) {return false;};
  scheduler.submit(std::move(stale));
  const int before = callbacks.load();
  scheduler.start();
  ASSERT_TRUE(wait_until([&]() {return scheduler.stats().dropped == 1U;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_GT(callbacks.load(), before + 1);
  EXPECT_TRUE(scheduler.stats().active_job.empty());
}

TEST(RuntimeScheduler, CpuWorkerCommitRechecksEpochCancelAndDeadline)
{
  BoundedCpuWorker worker(JobModule::ASR_CPU, 4, 100ms);
  std::atomic<bool> current{true};
  std::atomic<bool> cancelled{false};
  std::atomic<int> committed{0};
  auto value = cpu_job(JobModule::ASR_CPU, &committed);
  value.epoch_is_current = [&](uint64_t) {return current.load();};
  value.cancel_predicate = [&]() {return cancelled.load();};
  value.execute = [&](const CancellationToken &) {
      current.store(false);
      cancelled.store(true);
    };
  value.commit_if_current = [&](const CancellationToken & token) {
      if (!current.load() || cancelled.load()) {
        return false;
      }
      if (!token.try_begin_commit()) {
        return false;
      }
      ++committed;
      return true;
    };
  worker.submit(std::move(value));
  worker.start();
  ASSERT_TRUE(wait_until([&]() {return worker.stats().executed == 1U;}));
  EXPECT_TRUE(worker.stop());
  EXPECT_EQ(committed.load(), 0);
}

TEST(RuntimeScheduler, CpuWorkerRunningDeadlineSuppressesCommit)
{
  BoundedCpuWorker worker(JobModule::TTS_CPU, 2, 100ms);
  std::atomic<int> committed{0};
  auto value = cpu_job(JobModule::TTS_CPU, &committed);
  value.deadline = std::chrono::steady_clock::now() + 5ms;
  const auto deadline = value.deadline;
  value.execute = [](const CancellationToken &) {std::this_thread::sleep_for(15ms);};
  value.commit_if_current = [deadline, &committed](const CancellationToken & token) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      if (!token.try_begin_commit()) {
        return false;
      }
      ++committed;
      return true;
    };
  worker.submit(std::move(value));
  worker.start();
  ASSERT_TRUE(wait_until([&]() {return worker.stats().executed == 1U;}));
  EXPECT_TRUE(worker.stop());
  EXPECT_EQ(committed.load(), 0);
}

TEST(RuntimeScheduler, CpuCooperativeBlockingJobStopsWithinBound)
{
  BoundedCpuWorker worker(JobModule::VAD_CPU, 2, 100ms);
  std::atomic<bool> exited{false};
  auto value = cpu_job(JobModule::VAD_CPU);
  value.execute = [&](const CancellationToken & token) {
      while (!token.stop_requested()) {
        std::this_thread::sleep_for(1ms);
      }
      exited.store(true);
    };
  worker.submit(std::move(value));
  worker.start();
  ASSERT_TRUE(wait_until([&]() {return !worker.stats().active_job.empty();}));
  const auto before = std::chrono::steady_clock::now();
  EXPECT_TRUE(worker.stop());
  EXPECT_LT(std::chrono::steady_clock::now() - before, 150ms);
  EXPECT_TRUE(exited.load());
}

TEST(RuntimeScheduler, CpuStopTimeoutReportsFailureAndCanBeRetried)
{
  BoundedCpuWorker worker(JobModule::VISION_CPU, 2, 0ms);
  std::atomic<bool> exited{false};
  auto value = cpu_job(JobModule::VISION_CPU);
  value.execute = [&](const CancellationToken & token) {
      while (!token.stop_requested()) {
        std::this_thread::sleep_for(1ms);
      }
      std::this_thread::sleep_for(5ms);
      exited.store(true);
    };
  worker.submit(std::move(value));
  worker.start();
  ASSERT_TRUE(wait_until([&]() {return !worker.stats().active_job.empty();}));
  const auto before = std::chrono::steady_clock::now();
  EXPECT_FALSE(worker.stop());
  EXPECT_LT(std::chrono::steady_clock::now() - before, 20ms);
  EXPECT_TRUE(worker.has_live_thread());
  std::this_thread::sleep_for(15ms);
  EXPECT_TRUE(worker.stop());
  EXPECT_FALSE(worker.has_live_thread());
  EXPECT_TRUE(exited.load());
}

TEST(RuntimeScheduler, CpuWorkersAreIsolatedByModule)
{
  BoundedCpuWorker wakeword(JobModule::WAKEWORD_CPU, 1, 100ms);
  BoundedCpuWorker vad(JobModule::VAD_CPU, 1, 100ms);
  EXPECT_TRUE(wakeword.submit(cpu_job(JobModule::WAKEWORD_CPU)));
  EXPECT_FALSE(wakeword.submit(cpu_job(JobModule::VAD_CPU)));
  EXPECT_TRUE(vad.submit(cpu_job(JobModule::VAD_CPU)));
  wakeword.start();
  vad.start();
  ASSERT_TRUE(wait_until(
    [&]() {return wakeword.stats().committed == 1U && vad.stats().committed == 1U;}));
  EXPECT_TRUE(vad.stop());
  EXPECT_TRUE(wakeword.stop());
}

TEST(CancellationToken, CancelWinsBeforeCommitNeverCommits)
{
  CancellationToken token;
  EXPECT_TRUE(token.request_cancel());
  EXPECT_FALSE(token.try_begin_commit());
  EXPECT_EQ(token.state(), CancellationState::Cancelled);
}

TEST(CancellationToken, CommitWinsBeforeCancelCompletesOnce)
{
  CancellationToken token;
  EXPECT_TRUE(token.try_begin_commit());
  EXPECT_FALSE(token.request_cancel());
  EXPECT_EQ(token.state(), CancellationState::Committing);
  token.mark_completed();
  EXPECT_EQ(token.state(), CancellationState::Completed);
}

TEST(CancellationToken, ConcurrentCancelVsCommitHasSingleWinner)
{
  for (int iteration = 0; iteration < 100; ++iteration) {
    CancellationToken token;
    std::mutex mutex;
    std::condition_variable condition;
    int ready = 0;
    bool release = false;
    std::atomic<bool> cancelled{false};
    std::atomic<bool> committing{false};
    auto wait_at_barrier = [&]() {
        std::unique_lock<std::mutex> lock(mutex);
        ++ready;
        condition.notify_all();
        EXPECT_TRUE(condition.wait_for(lock, 500ms, [&]() {return release;}));
      };
    std::thread cancel_thread([&]() {
      wait_at_barrier();
      cancelled.store(token.request_cancel());
    });
    std::thread commit_thread([&]() {
      wait_at_barrier();
      committing.store(token.try_begin_commit());
      if (committing.load()) {
        token.mark_completed();
      }
    });
    bool both_ready = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      both_ready = condition.wait_for(lock, 500ms, [&]() {return ready == 2;});
      release = true;
    }
    condition.notify_all();
    cancel_thread.join();
    commit_thread.join();
    ASSERT_TRUE(both_ready);
    EXPECT_NE(cancelled.load(), committing.load());
    EXPECT_TRUE(
      token.state() == CancellationState::Cancelled ||
      token.state() == CancellationState::Completed);
  }
}

TEST(CommitGenerationGate, GenerationUpdateWaitsForAtomicCommit)
{
  CommitGenerationGate gate;
  ASSERT_TRUE(gate.update(7, true, true, true, true));
  CancellationToken token;
  std::mutex mutex;
  std::condition_variable condition;
  bool commit_started = false;
  bool release_commit = false;
  std::atomic<bool> update_finished{false};

  std::thread commit_thread([&]() {
    EXPECT_TRUE(gate.commit_if_current(
      7, JobModule::ASR_EP, std::chrono::steady_clock::now() + 1s, token,
      [&]() {
        std::unique_lock<std::mutex> lock(mutex);
        commit_started = true;
        condition.notify_all();
        EXPECT_TRUE(condition.wait_for(lock, 500ms, [&]() {return release_commit;}));
      }));
  });
  bool observed_commit = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    observed_commit = condition.wait_for(lock, 500ms, [&]() {return commit_started;});
  }
  if (!observed_commit) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      release_commit = true;
    }
    condition.notify_all();
    commit_thread.join();
    ASSERT_TRUE(observed_commit);
  }
  std::thread update_thread([&]() {
    gate.update(8, false, false, false, false);
    update_finished.store(true);
  });
  std::this_thread::sleep_for(5ms);
  EXPECT_FALSE(update_finished.load());
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_commit = true;
  }
  condition.notify_all();
  commit_thread.join();
  update_thread.join();
  EXPECT_TRUE(update_finished.load());
  EXPECT_EQ(token.state(), CancellationState::Completed);
}

TEST(RuntimeScheduler, CommitExceptionMarksCompletedAndWorkerContinues)
{
  ResourceGuard guard;
  RuntimeScheduler scheduler(guard, 4, 100ms);
  CancellationToken captured;
  auto failing = guarded_job("commit-throws", JobModule::ASR_EP);
  failing.commit_if_current = [&](const CancellationToken & token) {
      captured = token;
      if (!token.try_begin_commit()) {
        return false;
      }
      throw std::runtime_error("commit failure");
    };
  std::atomic<int> next_committed{0};
  auto next = guarded_job("next-after-commit-failure", JobModule::VISION_EP);
  next.commit_if_current = [&](const CancellationToken & token) {
      if (!token.try_begin_commit()) {
        return false;
      }
      ++next_committed;
      return true;
    };
  scheduler.submit(std::move(failing));
  scheduler.submit(std::move(next));
  scheduler.start();
  ASSERT_TRUE(wait_until(
    [&]() {return scheduler.stats().failed == 1U && next_committed.load() == 1;}));
  EXPECT_TRUE(scheduler.stop());
  EXPECT_EQ(captured.state(), CancellationState::Completed);
}
