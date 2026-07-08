#include "k1muse_control_manager/grasp_task_executor.hpp"

#include <chrono>
#include <sstream>
#include <thread>

namespace k1muse_control_manager
{

GraspTaskExecutor::GraspTaskExecutor(
  Stm32ActuatorClient & client, DeviceSession & session)
: client_(client), session_(session)
{
}

TaskResult GraspTaskExecutor::execute_pick(const Request & request)
{
  k1_pickup_plan_t plan{};
  const auto target = target_point_from_request(request);
  const auto plan_status = k1_plan_pickup(
    &target, &request.geometry, &request.profile, &plan);
  if (plan_status != K1_PLAN_OK) {
    std::ostringstream oss;
    oss << "pickup planning failed, status=" << static_cast<int>(plan_status);
    return make_failure(oss.str());
  }
  if (plan.lift_target_z < request.geometry.lift_min_z ||
      plan.lift_target_z > request.geometry.lift_max_z) {
    std::ostringstream oss;
    oss << "lift target out of range: " << plan.lift_target_z;
    return make_failure(oss.str());
  }

  auto result = client_.stop_all();
  if (!result.success) {return result;}

  result = session_.ensure_arm_ready();
  if (!result.success) {return result;}

  result = session_.ensure_lift_ready();
  if (!result.success) {return result;}

  result = client_.lift_move_z(plan.lift_target_z);
  if (!result.success) {return result;}

  result = client_.arm_gripper(request.profile.gripper_open_rad);
  if (!result.success) {return result;}

  result = move_pose(plan.pregrasp_pose);
  if (!result.success) {return result;}

  result = move_pose(plan.grasp_pose);
  if (!result.success) {return result;}

  result = client_.arm_gripper(request.profile.gripper_close_rad);
  if (!result.success) {return result;}

  std::this_thread::sleep_for(
    std::chrono::milliseconds(request.profile.settle_time_ms));

  if (request.carry_pose) {
    result = move_pose(*request.carry_pose);
  } else {
    result = move_pose(plan.lifted_pose);
    if (!result.success) {return result;}
    result = move_pose(plan.retreat_pose);
  }
  if (!result.success) {return result;}

  return make_success("pick complete, holding");
}

TaskResult GraspTaskExecutor::execute_place(const Request & request)
{
  k1_place_plan_t plan{};
  const auto target = target_point_from_request(request);
  const auto plan_status = k1_plan_place(
    &target, &request.geometry, &request.profile, &plan);
  if (plan_status != K1_PLAN_OK) {
    std::ostringstream oss;
    oss << "place planning failed, status=" << static_cast<int>(plan_status);
    return make_failure(oss.str());
  }
  if (plan.lift_target_z < request.geometry.lift_min_z ||
      plan.lift_target_z > request.geometry.lift_max_z) {
    std::ostringstream oss;
    oss << "lift target out of range: " << plan.lift_target_z;
    return make_failure(oss.str());
  }

  auto result = client_.stop_all();
  if (!result.success) {return result;}

  result = session_.ensure_arm_ready();
  if (!result.success) {return result;}

  result = session_.ensure_lift_ready();
  if (!result.success) {return result;}

  result = client_.lift_move_z(plan.lift_target_z);
  if (!result.success) {return result;}

  result = move_pose(plan.preplace_pose);
  if (!result.success) {return result;}

  result = move_pose(plan.approach_pose);
  if (!result.success) {return result;}

  result = move_pose(plan.place_pose);
  if (!result.success) {return result;}

  result = client_.arm_gripper(request.profile.gripper_open_rad);
  if (!result.success) {return result;}

  std::this_thread::sleep_for(
    std::chrono::milliseconds(request.profile.settle_time_ms));

  result = move_pose(plan.retreat_pose);
  if (!result.success) {return result;}

  if (request.carry_pose) {
    result = move_pose(*request.carry_pose);
    if (!result.success) {return result;}
  }

  return make_success("place complete, released");
}

TaskResult GraspTaskExecutor::execute_grasp(const Request & request)
{
  return execute_pick(request);
}

TaskResult GraspTaskExecutor::release(float open_angle_rad)
{
  return client_.arm_gripper(open_angle_rad);
}

k1_point3f_t GraspTaskExecutor::target_point_from_request(const Request & request)
{
  k1_point3f_t target{};
  target.x = request.geometry.arm_mount_x + request.profile.preferred_arm_x;
  target.y = request.geometry.arm_mount_y + request.profile.preferred_arm_y;
  target.z = request.target_z;
  return target;
}

TaskResult GraspTaskExecutor::move_pose(const k1_pose5f_t & pose)
{
  return client_.arm_move_pose(pose.x, pose.y, pose.z, pose.roll, pose.pitch);
}

}  // namespace k1muse_control_manager
