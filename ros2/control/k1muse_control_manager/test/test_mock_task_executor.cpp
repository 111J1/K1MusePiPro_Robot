#include <gtest/gtest.h>

#include "k1muse_control_manager/mock_task_executor.hpp"

using k1muse_control_manager::MockTaskExecutor;
using k1muse_control_manager::TaskResult;

TEST(MockTaskExecutorTest, MoveSuccess)
{
  MockTaskExecutor executor;
  TaskResult result = executor.execute_move("forward", 10);
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.reason.empty());
}

TEST(MockTaskExecutorTest, StopSuccess)
{
  MockTaskExecutor executor;
  TaskResult result = executor.execute_stop(10);
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.reason.empty());
}

TEST(MockTaskExecutorTest, LiftSuccess)
{
  MockTaskExecutor executor;
  TaskResult result = executor.execute_lift("up", 10);
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.reason.empty());
}

TEST(MockTaskExecutorTest, RotateSuccess)
{
  MockTaskExecutor executor;
  TaskResult result = executor.execute_rotate("left", 10);
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.reason.empty());
}
