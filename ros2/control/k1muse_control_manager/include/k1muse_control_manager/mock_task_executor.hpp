#pragma once

#include <string>

#include "k1muse_control_manager/task_result.hpp"

namespace k1muse_control_manager
{

class MockTaskExecutor
{
public:
  MockTaskExecutor() = default;

  /// Mock move: sleep for duration_ms, return success.
  TaskResult execute_move(const std::string & direction, int duration_ms);

  /// Mock stop: sleep for duration_ms, return success.
  TaskResult execute_stop(int duration_ms);

  /// Mock lift: sleep for duration_ms, return success.
  TaskResult execute_lift(const std::string & direction, int duration_ms);

  /// Mock rotate: sleep for duration_ms, return success.
  TaskResult execute_rotate(const std::string & direction, int duration_ms);
};

}  // namespace k1muse_control_manager
