#pragma once

#include <chrono>
#include <string>

namespace k1muse_control_manager
{

enum class ExecutionPolicyMode
{
  Strict,
  DemoTolerant,
};

struct ExecutionPolicy
{
  ExecutionPolicyMode mode = ExecutionPolicyMode::DemoTolerant;
  std::chrono::milliseconds result_timeout{15000};
  std::chrono::milliseconds gripper_timeout{1000};
  float arm_pose_xyz_tolerance_m = 0.015f;
  float lift_z_tolerance_m = 0.02f;
  float gripper_tolerance_rad = 0.05f;
  bool assume_gripper_success_without_fault = true;
  bool rehome_lift_on_fault = true;
};

ExecutionPolicyMode parse_execution_policy_mode(const std::string & value);
std::string to_string(ExecutionPolicyMode mode);

}  // namespace k1muse_control_manager
