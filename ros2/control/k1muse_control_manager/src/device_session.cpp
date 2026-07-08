#include "k1muse_control_manager/device_session.hpp"

#include <chrono>

namespace k1muse_control_manager
{

namespace
{

constexpr uint8_t kArmStateIdle = 3;
constexpr uint8_t kArmStateWaitTarget = 5;
constexpr uint8_t kArmStateReached = 6;

}  // namespace

DeviceSession::DeviceSession(Stm32ActuatorClient & client)
: client_(client)
{
}

TaskResult DeviceSession::ensure_arm_ready()
{
  auto status = client_.wait_for_arm_status(std::chrono::milliseconds(1000));
  if (status && status->is_busy != 0U) {
    status = client_.wait_for_arm_status(std::chrono::milliseconds(1500));
  }
  if (status && is_arm_ready(*status)) {
    return make_success("arm ready, home skipped");
  }
  return client_.arm_home();
}

TaskResult DeviceSession::ensure_lift_ready()
{
  auto status = client_.wait_for_lift_status(std::chrono::milliseconds(1000));
  if (status && status->has_fault != 0U) {
    auto result = client_.lift_clear_fault();
    if (!result.success) {
      return result;
    }
    return client_.lift_home();
  }
  if (status && status->has_fault == 0U && status->is_homed != 0U) {
    return make_success("lift homed, home skipped");
  }
  return client_.lift_home();
}

bool DeviceSession::is_arm_ready(const k1_arm_status_t & status)
{
  if (status.has_fault != 0U || status.is_busy != 0U) {
    return false;
  }
  return status.state == kArmStateIdle ||
         status.state == kArmStateWaitTarget ||
         status.state == kArmStateReached;
}

}  // namespace k1muse_control_manager
