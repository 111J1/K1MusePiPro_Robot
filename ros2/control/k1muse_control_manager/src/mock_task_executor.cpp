#include "k1muse_control_manager/mock_task_executor.hpp"

#include <chrono>
#include <thread>

namespace k1muse_control_manager
{

TaskResult MockTaskExecutor::execute_move(const std::string & direction, int duration_ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  TaskResult result;
  result.success = true;
  result.reason = "Mock move " + direction + " completed";
  return result;
}

TaskResult MockTaskExecutor::execute_stop(int duration_ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  TaskResult result;
  result.success = true;
  result.reason = "Mock stop completed";
  return result;
}

TaskResult MockTaskExecutor::execute_lift(const std::string & direction, int duration_ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  TaskResult result;
  result.success = true;
  result.reason = "Mock lift " + direction + " completed";
  return result;
}

TaskResult MockTaskExecutor::execute_rotate(const std::string & direction, int duration_ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  TaskResult result;
  result.success = true;
  result.reason = "Mock rotate " + direction + " completed";
  return result;
}

}  // namespace k1muse_control_manager
