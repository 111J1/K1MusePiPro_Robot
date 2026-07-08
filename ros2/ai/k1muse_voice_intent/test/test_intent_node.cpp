#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_voice_intent/intent_node.hpp"
#include "k1muse_voice_msgs/msg/intent_lite.hpp"
#include "k1muse_voice_msgs/msg/intent_status.hpp"
#include "k1muse_voice_msgs/msg/listen_result.hpp"
#include "k1muse_voice_msgs/msg/tts_text_request.hpp"

namespace k1muse_voice_intent
{
namespace
{

using ListenResult = k1muse_voice_msgs::msg::ListenResult;
using IntentLite = k1muse_voice_msgs::msg::IntentLite;
using IntentStatus = k1muse_voice_msgs::msg::IntentStatus;
using TtsTextRequest = k1muse_voice_msgs::msg::TtsTextRequest;

// ---- Helpers ----

// Generic message collector with thread-safe storage and wait support.
template <typename T>
class MessageCollector
{
public:
  explicit MessageCollector(
    rclcpp::Node::SharedPtr node, const std::string & topic,
    const rclcpp::QoS & qos = rclcpp::QoS(10).reliable().durability_volatile())
  {
    sub_ = node->create_subscription<T>(
      topic, qos,
      [this](typename T::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        msgs_.push_back(*msg);
        cv_.notify_all();
      });
  }

  // Wait until at least `count` messages have been received.
  bool wait_for(size_t count, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    return cv_.wait_for(lock, timeout, [this, count]() {
      return msgs_.size() >= count;
    });
  }

  const std::vector<T> & messages() const { return msgs_; }
  size_t size() const { return msgs_.size(); }

private:
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  std::vector<T> msgs_;
  std::mutex mtx_;
  std::condition_variable cv_;
};

// Helper: create a ListenResult message.
ListenResult make_listen_result(
  const std::string & text, bool success = true, uint64_t epoch = 1,
  const std::string & trace_id = "t1", const std::string & utterance_id = "u1")
{
  ListenResult msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.trace_id = trace_id;
  msg.utterance_id = utterance_id;
  msg.epoch = epoch;
  msg.success = success;
  msg.text = text;
  msg.confidence = 1.0f;
  msg.language = "zh";
  return msg;
}

class BlockingLlmClient : public LlmIntentClient
{
public:
  bool health_check(std::string * reason = nullptr) override
  {
    (void)reason;
    return true;
  }

  bool warmup(std::string * reason = nullptr) override
  {
    (void)reason;
    return true;
  }

  LlmResult complete_intent(
    const std::string & /*text*/, const LlmRequestContext & context) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++call_count_;
      entered_ = true;
      cv_.notify_all();
    }

    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&]() {
      return released_ ||
        (context.cancelled && context.cancelled->load(std::memory_order_acquire));
    });

    LlmResult result;
    if (context.cancelled && context.cancelled->load(std::memory_order_acquire)) {
      result.status = LlmStatus::kCancelled;
      result.error = "cancelled";
      return result;
    }
    result.status = LlmStatus::kOk;
    result.http_status = 200;
    result.content = R"({"kind":"query_introduce","direction":"","target":"","reply":"ok"})";
    return result;
  }

  const std::string & name() const override { return name_; }

  bool wait_until_entered(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&]() { return entered_; });
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    cv_.notify_all();
  }

  int call_count() const { return call_count_.load(); }

private:
  std::string name_{"blocking"};
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool entered_{false};
  bool released_{false};
  std::atomic<int> call_count_{0};
};

class DeadlineCapturingLlmClient : public LlmIntentClient
{
public:
  bool health_check(std::string * reason = nullptr) override
  {
    (void)reason;
    return true;
  }

  bool warmup(std::string * reason = nullptr) override
  {
    (void)reason;
    return true;
  }

  LlmResult complete_intent(
    const std::string & /*text*/, const LlmRequestContext & context) override
  {
    const auto now = std::chrono::steady_clock::now();
    captured_timeout_ms_ =
      std::chrono::duration_cast<std::chrono::milliseconds>(context.deadline - now).count();

    LlmResult result;
    result.status = LlmStatus::kOk;
    result.http_status = 200;
    result.content = R"({"kind":"query_introduce","direction":"","target":"","reply":"ok"})";
    return result;
  }

  const std::string & name() const override { return name_; }
  int64_t captured_timeout_ms() const { return captured_timeout_ms_.load(); }

private:
  std::string name_{"deadline_capture"};
  std::atomic<int64_t> captured_timeout_ms_{-1};
};
// ---- Global test environment: init rclcpp once ----

class IntentNodeTestEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

::testing::Environment * const kEnv =
  ::testing::AddGlobalTestEnvironment(new IntentNodeTestEnvironment);

// ---- Test fixture (default: busy_policy=reject) ----

class IntentNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<IntentNode>(rclcpp::NodeOptions());
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    spin_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_future_.valid()) {
      spin_future_.wait();
    }
    executor_->remove_node(node_);
    node_.reset();
    executor_.reset();
  }

  std::shared_ptr<IntentNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::future<void> spin_future_;
};

// Fixture variant with busy_policy=replace (for stale-epoch tests).
class IntentNodeReplaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("busy_policy", "replace")
    });
    node_ = std::make_shared<IntentNode>(options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    spin_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_future_.valid()) {
      spin_future_.wait();
    }
    executor_->remove_node(node_);
    node_.reset();
    executor_.reset();
  }

  std::shared_ptr<IntentNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::future<void> spin_future_;
};

// ---- Tests ----

// Test 1: FastHitPublishesIntent
// "前进" matches fast intent -> IntentLite published with action=move.
TEST_F(IntentNodeTest, FastHitPublishesIntent)
{
  MessageCollector<IntentLite> intent_col(node_, "/voice/intent");

  node_->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("前进")));

  ASSERT_TRUE(intent_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_EQ(intent_col.messages()[0].action, "move");
  EXPECT_EQ(intent_col.messages()[0].target, "chassis");
  EXPECT_EQ(intent_col.messages()[0].value, "forward");
}

// Test 2: PublicationOrder
// Verify ACTIVE -> IntentLite -> FINISHED ordering.
TEST_F(IntentNodeTest, PublicationOrder)
{
  struct Msg
  {
    std::string type;
    uint8_t status_state{0};
  };
  std::vector<Msg> order;
  std::mutex order_mutex;
  std::condition_variable order_cv;

  node_->create_subscription<IntentStatus>(
    "/voice/intent/status", rclcpp::QoS(10).reliable().durability_volatile(),
    [&](IntentStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(order_mutex);
      Msg m;
      m.type = "status";
      m.status_state = msg->state;
      order.push_back(m);
      order_cv.notify_all();
    });

  node_->create_subscription<IntentLite>(
    "/voice/intent", rclcpp::QoS(10).reliable().durability_volatile(),
    [&](IntentLite::SharedPtr) {
      std::lock_guard<std::mutex> lock(order_mutex);
      Msg m;
      m.type = "intent";
      order.push_back(m);
      order_cv.notify_all();
    });

  node_->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("前进")));

  {
    std::unique_lock<std::mutex> lock(order_mutex);
    ASSERT_TRUE(order_cv.wait_for(lock, std::chrono::milliseconds(3000), [&]() {
      bool has_active = false, has_finished = false;
      for (const auto & m : order) {
        if (m.type == "status" && m.status_state == IntentStatus::STATE_ACTIVE) has_active = true;
        if (m.type == "status" && m.status_state == IntentStatus::STATE_FINISHED) has_finished = true;
      }
      return has_active && has_finished;
    }));
  }

  ASSERT_GE(order.size(), 3u);
  size_t active_idx = order.size(), intent_idx = order.size(), finished_idx = order.size();
  for (size_t i = 0; i < order.size(); ++i) {
    if (order[i].type == "status" && order[i].status_state == IntentStatus::STATE_ACTIVE)
      active_idx = i;
    if (order[i].type == "intent")
      intent_idx = i;
    if (order[i].type == "status" && order[i].status_state == IntentStatus::STATE_FINISHED)
      finished_idx = i;
  }
  EXPECT_LT(active_idx, intent_idx);
  EXPECT_LT(intent_idx, finished_idx);
}

// Test 3: TtsReplyPublished
// Fast intent "前进" has tts_reply -> TtsTextRequest published.
TEST_F(IntentNodeTest, TtsReplyPublished)
{
  MessageCollector<TtsTextRequest> tts_col(node_, "/voice/tts/text");

  node_->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("前进")));

  ASSERT_TRUE(tts_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_FALSE(tts_col.messages()[0].text.empty());
  EXPECT_EQ(tts_col.messages()[0].priority, static_cast<uint8_t>(1));
  EXPECT_EQ(tts_col.messages()[0].source, "intent_router");
}

// Test 4: EmptyTextFailed
// Empty text -> FAILED IntentStatus published.
TEST_F(IntentNodeTest, EmptyTextFailed)
{
  MessageCollector<IntentStatus> status_col(node_, "/voice/intent/status");

  ListenResult msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.trace_id = "t1";
  msg.utterance_id = "u1";
  msg.epoch = 1;
  msg.success = true;
  msg.text = "";
  node_->on_listen_result(std::make_shared<ListenResult>(msg));

  ASSERT_TRUE(status_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_EQ(status_col.messages()[0].state, IntentStatus::STATE_FAILED);
  EXPECT_EQ(status_col.messages()[0].reason, "empty text");
}

// Test 5: BusyRejectsSecond
// A rejected request must publish an explicit FAILED status instead of
// disappearing silently.
TEST(IntentNodeCustomClientTest, BusyRejectPublishesFailedStatus)
{
  RouterConfig config;
  config.allow_fast_intent = false;
  config.llm_fallback_enabled = true;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("busy_policy", "reject")
  });

  auto client = std::make_unique<BlockingLlmClient>();
  auto * client_ptr = client.get();
  auto node = std::make_shared<IntentNode>(options, config, std::move(client));
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node->get_node_base_interface());
  auto spin_future = std::async(std::launch::async, [&]() { executor->spin(); });

  MessageCollector<IntentStatus> status_col(node, "/voice/intent/status");

  node->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("first request", true, 1)));
  ASSERT_TRUE(client_ptr->wait_until_entered(std::chrono::milliseconds(3000)));

  node->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("second request", true, 2)));

  ASSERT_TRUE(status_col.wait_for(2, std::chrono::milliseconds(3000)));
  bool saw_busy_reject = false;
  for (const auto & status : status_col.messages()) {
    if (status.epoch == 2 && status.state == IntentStatus::STATE_FAILED) {
      saw_busy_reject = true;
      EXPECT_EQ(status.reason, "busy");
      EXPECT_FALSE(status.has_tts);
    }
  }
  EXPECT_TRUE(saw_busy_reject);
  EXPECT_EQ(client_ptr->call_count(), 1);

  client_ptr->release();
  ASSERT_TRUE(status_col.wait_for(3, std::chrono::milliseconds(3000)));
  executor->cancel();
  spin_future.wait();
  executor->remove_node(node);
}

TEST(IntentNodeCustomClientTest, CancelledStatusHasNoTts)
{
  RouterConfig config;
  config.allow_fast_intent = false;
  config.llm_fallback_enabled = true;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("busy_policy", "replace")
  });

  auto client = std::make_unique<BlockingLlmClient>();
  auto * client_ptr = client.get();
  auto node = std::make_shared<IntentNode>(options, config, std::move(client));
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node->get_node_base_interface());
  auto spin_future = std::async(std::launch::async, [&]() { executor->spin(); });

  MessageCollector<IntentStatus> status_col(node, "/voice/intent/status");

  node->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("first request", true, 300)));
  ASSERT_TRUE(client_ptr->wait_until_entered(std::chrono::milliseconds(3000)));

  node->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("second request", true, 301)));

  ASSERT_TRUE(status_col.wait_for(1, std::chrono::milliseconds(3000)));
  bool saw_cancelled = false;
  for (const auto & status : status_col.messages()) {
    if (status.epoch == 300 && status.state == IntentStatus::STATE_FAILED &&
        status.reason == "cancelled") {
      saw_cancelled = true;
      EXPECT_FALSE(status.has_tts);
    }
  }
  EXPECT_TRUE(saw_cancelled);

  client_ptr->release();
  ASSERT_TRUE(status_col.wait_for(3, std::chrono::milliseconds(3000)));
  executor->cancel();
  spin_future.wait();
  executor->remove_node(node);
}
TEST(IntentNodeCustomClientTest, LlmDeadlineUsesConfiguredTimeout)
{
  RouterConfig config;
  config.allow_fast_intent = false;
  config.llm_fallback_enabled = true;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("llm_request_timeout_ms", 1234)
  });

  auto client = std::make_unique<DeadlineCapturingLlmClient>();
  auto * client_ptr = client.get();
  auto node = std::make_shared<IntentNode>(options, config, std::move(client));
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node->get_node_base_interface());
  auto spin_future = std::async(std::launch::async, [&]() { executor->spin(); });

  MessageCollector<IntentStatus> status_col(node, "/voice/intent/status");

  node->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("deadline request", true, 10)));

  ASSERT_TRUE(status_col.wait_for(2, std::chrono::milliseconds(3000)));
  EXPECT_GE(client_ptr->captured_timeout_ms(), 1000);
  EXPECT_LE(client_ptr->captured_timeout_ms(), 1234);

  executor->cancel();
  spin_future.wait();
  executor->remove_node(node);
}

// Test 6: StaleEpochDropped
// With busy_policy=replace, a new request interrupts the current worker.
// The first worker sees stale epoch and drops its result.
TEST_F(IntentNodeReplaceTest, StaleEpochDropped)
{
  MessageCollector<IntentLite> intent_col(node_, "/voice/intent");

  // Send first request (epoch=100).
  node_->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("前进", true, 100)));

  // Send second request (epoch=200). With replace mode, this will:
  // 1. Update latest_epoch to 200
  // 2. Join the first worker (which drops its result due to stale epoch)
  // 3. Launch a new worker for the second request
  node_->on_listen_result(
    std::make_shared<ListenResult>(make_listen_result("后退", true, 200)));

  // Should receive exactly one IntentLite from the second request ("后退").
  // The first request's result was dropped (stale epoch).
  ASSERT_TRUE(intent_col.wait_for(1, std::chrono::milliseconds(3000)));
  EXPECT_EQ(intent_col.size(), 1u);
  EXPECT_EQ(intent_col.messages()[0].value, "backward");
}

}  // namespace
}  // namespace k1muse_voice_intent
