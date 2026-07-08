#pragma once

#include <optional>

#include "k1muse_control_core/grasp_profile.h"
#include "k1muse_control_core/placement_planner.h"

#include "k1muse_control_manager/device_session.hpp"
#include "k1muse_control_manager/task_result.hpp"

namespace k1muse_control_manager
{

class GraspTaskExecutor
{
public:
  struct Request
  {
    k1_robot_geometry_t geometry{};
    k1_grasp_variant_t profile{};
    float target_z = 0.0f;
    std::optional<k1_pose5f_t> carry_pose;
  };

  GraspTaskExecutor(Stm32ActuatorClient & client, DeviceSession & session);

  TaskResult execute_pick(const Request & request);
  TaskResult execute_place(const Request & request);
  TaskResult execute_grasp(const Request & request);
  TaskResult release(float open_angle_rad);

private:
  static k1_point3f_t target_point_from_request(const Request & request);
  TaskResult move_pose(const k1_pose5f_t & pose);

  Stm32ActuatorClient & client_;
  DeviceSession & session_;
};

}  // namespace k1muse_control_manager
