#include "k1muse_control_manager/execution_policy.hpp"

#include <stdexcept>

namespace k1muse_control_manager
{

ExecutionPolicyMode parse_execution_policy_mode(const std::string & value)
{
  if (value == "strict") {
    return ExecutionPolicyMode::Strict;
  }
  if (value == "demo_tolerant") {
    return ExecutionPolicyMode::DemoTolerant;
  }
  throw std::invalid_argument("unknown execution policy: " + value);
}

std::string to_string(ExecutionPolicyMode mode)
{
  switch (mode) {
    case ExecutionPolicyMode::Strict:
      return "strict";
    case ExecutionPolicyMode::DemoTolerant:
      return "demo_tolerant";
  }
  return "unknown";
}

}  // namespace k1muse_control_manager
