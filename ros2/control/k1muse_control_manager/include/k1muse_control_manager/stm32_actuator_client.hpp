#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "k1muse_control_core/actuator_protocol.h"

#include "k1muse_control_manager/execution_policy.hpp"
#include "k1muse_control_manager/serial_transport.hpp"
#include "k1muse_control_manager/task_result.hpp"

namespace k1muse_control_manager
{

class Stm32ActuatorClient
{
public:
  struct Options
  {
    uint8_t source = K1_CTRL_SRC_HOST;
    ExecutionPolicy policy;
    std::chrono::milliseconds read_timeout{20};
  };

  Stm32ActuatorClient(std::unique_ptr<ByteTransport> transport, Options options);
  ~Stm32ActuatorClient();

  void start();
  void stop();

  TaskResult stop_all();
  TaskResult arm_home();
  TaskResult arm_move_pose(float x, float y, float z, float roll, float pitch);
  TaskResult arm_gripper(float angle_rad);
  TaskResult lift_home();
  TaskResult lift_clear_fault();
  TaskResult lift_move_z(float z_m);

  std::optional<k1_arm_status_t> latest_arm_status() const;
  std::optional<k1_lift_status_t> latest_lift_status() const;
  std::optional<k1_chassis_status_t> latest_chassis_status() const;

  std::optional<k1_arm_status_t> wait_for_arm_status(std::chrono::milliseconds timeout);
  std::optional<k1_lift_status_t> wait_for_lift_status(std::chrono::milliseconds timeout);

private:
  struct ResultEnvelope
  {
    uint8_t target = 0;
    k1_arm_result_t arm{};
    k1_lift_result_t lift{};
  };

  uint8_t next_seq();
  void send_frame(k1_ctrl_frame_t & frame);
  TaskResult wait_command_result(uint8_t target, uint8_t seq,
    std::chrono::milliseconds timeout);
  void read_loop();
  void process_frame(const k1_ctrl_frame_t & frame);
  TaskResult apply_arm_pose_tolerance(const TaskResult & failure,
    float x, float y, float z);
  TaskResult apply_gripper_tolerance_or_policy(const TaskResult & failure,
    float angle_rad);

  std::unique_ptr<ByteTransport> transport_;
  Options options_;
  std::atomic<bool> running_{false};
  std::thread read_thread_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  k1_ctrl_parser_t parser_{};
  uint8_t seq_{0};

  std::optional<k1_arm_status_t> latest_arm_status_;
  std::optional<k1_lift_status_t> latest_lift_status_;
  std::optional<k1_chassis_status_t> latest_chassis_status_;
  std::vector<ResultEnvelope> results_;
};

}  // namespace k1muse_control_manager
