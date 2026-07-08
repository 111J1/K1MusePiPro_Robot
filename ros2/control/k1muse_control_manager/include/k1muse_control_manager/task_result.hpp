#pragma once

#include <string>

namespace k1muse_control_manager
{

struct TaskResult
{
  bool success = false;
  std::string reason;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline TaskResult make_success(const std::string & reason = "completed")
{
  TaskResult result;
  result.success = true;
  result.reason = reason;
  return result;
}

inline TaskResult make_failure(const std::string & reason)
{
  TaskResult result;
  result.success = false;
  result.reason = reason;
  return result;
}

}  // namespace k1muse_control_manager
