#pragma once

#include "k1muse_control_manager/stm32_actuator_client.hpp"
#include "k1muse_control_manager/task_result.hpp"

namespace k1muse_control_manager
{

class DeviceSession
{
public:
  explicit DeviceSession(Stm32ActuatorClient & client);

  TaskResult ensure_arm_ready();
  TaskResult ensure_lift_ready();

private:
  static bool is_arm_ready(const k1_arm_status_t & status);
  Stm32ActuatorClient & client_;
};

}  // namespace k1muse_control_manager
