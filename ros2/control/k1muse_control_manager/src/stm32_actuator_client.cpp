#include "k1muse_control_manager/stm32_actuator_client.hpp"

#include <cmath>
#include <array>
#include <sstream>
#include <stdexcept>

namespace k1muse_control_manager
{

namespace
{

double arm_xyz_error(const k1_arm_status_t & status, float x, float y, float z)
{
  const double dx = static_cast<double>(status.current_x - x);
  const double dy = static_cast<double>(status.current_y - y);
  const double dz = static_cast<double>(status.current_z - z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

TaskResult result_failure(uint8_t target, uint8_t seq, uint8_t result,
  uint8_t reject, uint8_t fault, uint8_t state_after)
{
  std::ostringstream oss;
  oss << "target=" << static_cast<int>(target)
      << " seq=" << static_cast<int>(seq)
      << " result=" << static_cast<int>(result)
      << " reject=" << static_cast<int>(reject)
      << " fault=" << static_cast<int>(fault)
      << " state_after=" << static_cast<int>(state_after);
  return make_failure(oss.str());
}

}  // namespace

Stm32ActuatorClient::Stm32ActuatorClient(
  std::unique_ptr<ByteTransport> transport, Options options)
: transport_(std::move(transport)), options_(options)
{
  if (!transport_) {
    throw std::invalid_argument("transport must not be null");
  }
  k1_ctrl_parser_init(&parser_);
}

Stm32ActuatorClient::~Stm32ActuatorClient()
{
  stop();
}

void Stm32ActuatorClient::start()
{
  if (running_) {
    return;
  }
  transport_->open();
  running_ = true;
  read_thread_ = std::thread([this]() { read_loop(); });
}

void Stm32ActuatorClient::stop()
{
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (transport_) {
    transport_->close();
  }
}

uint8_t Stm32ActuatorClient::next_seq()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const uint8_t value = seq_;
  seq_ = static_cast<uint8_t>(seq_ + 1U);
  return value;
}

void Stm32ActuatorClient::send_frame(k1_ctrl_frame_t & frame)
{
  frame.src = options_.source;
  uint8_t encoded[K1_CTRL_MAX_FRAME_SIZE]{};
  const size_t size = k1_ctrl_encode_frame(&frame, encoded, sizeof(encoded));
  if (size == 0U) {
    throw std::runtime_error("failed to encode control frame");
  }
  transport_->write_bytes(encoded, size);
}

TaskResult Stm32ActuatorClient::wait_command_result(
  uint8_t target, uint8_t seq, std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    for (auto it = results_.begin(); it != results_.end(); ++it) {
      if (it->target != target) {
        continue;
      }
      if (target == K1_CTRL_TARGET_ARM && it->arm.request_seq == seq) {
        const auto result = it->arm;
        results_.erase(it);
        if (result.result == K1_RESULT_ACCEPTED) {
          break;
        }
        if (result.result == K1_RESULT_COMPLETED) {
          return make_success("completed");
        }
        return result_failure(target, seq, result.result, result.reject_reason,
          result.fault_source, result.state_after);
      }
      if (target == K1_CTRL_TARGET_LIFT && it->lift.request_seq == seq) {
        const auto result = it->lift;
        results_.erase(it);
        if (result.result == K1_RESULT_ACCEPTED) {
          break;
        }
        if (result.result == K1_RESULT_COMPLETED) {
          return make_success("completed");
        }
        return result_failure(target, seq, result.result, result.reject_reason,
          result.fault_reason, result.state_after);
      }
    }
    cv_.wait_until(lock, deadline);
  }

  std::ostringstream oss;
  oss << "timed out waiting for target=" << static_cast<int>(target)
      << " seq=" << static_cast<int>(seq);
  return make_failure(oss.str());
}

TaskResult Stm32ActuatorClient::stop_all()
{
  const std::array<uint8_t, 3> targets = {
    static_cast<uint8_t>(K1_CTRL_TARGET_CHASSIS),
    static_cast<uint8_t>(K1_CTRL_TARGET_LIFT),
    static_cast<uint8_t>(K1_CTRL_TARGET_ARM)};
  for (const auto target : targets) {
    k1_ctrl_frame_t frame{};
    k1_build_stop_frame(&frame, target, next_seq());
    send_frame(frame);
  }
  return make_success("stop_all sent");
}

TaskResult Stm32ActuatorClient::arm_home()
{
  const uint8_t seq = next_seq();
  k1_ctrl_frame_t frame{};
  k1_build_arm_home_frame(&frame, seq);
  send_frame(frame);
  return wait_command_result(K1_CTRL_TARGET_ARM, seq, options_.policy.result_timeout);
}

TaskResult Stm32ActuatorClient::arm_move_pose(
  float x, float y, float z, float roll, float pitch)
{
  const uint8_t seq = next_seq();
  k1_ctrl_frame_t frame{};
  k1_build_arm_pose_frame(&frame, seq, x, y, z, roll, pitch);
  send_frame(frame);
  const auto result = wait_command_result(
    K1_CTRL_TARGET_ARM, seq, options_.policy.result_timeout);
  if (result.success || options_.policy.mode == ExecutionPolicyMode::Strict) {
    return result;
  }
  return apply_arm_pose_tolerance(result, x, y, z);
}

TaskResult Stm32ActuatorClient::arm_gripper(float angle_rad)
{
  const uint8_t seq = next_seq();
  k1_ctrl_frame_t frame{};
  k1_build_arm_gripper_frame(&frame, seq, angle_rad);
  send_frame(frame);
  const auto result = wait_command_result(
    K1_CTRL_TARGET_ARM, seq, options_.policy.gripper_timeout);
  if (result.success || options_.policy.mode == ExecutionPolicyMode::Strict) {
    return result;
  }
  return apply_gripper_tolerance_or_policy(result, angle_rad);
}

TaskResult Stm32ActuatorClient::lift_home()
{
  const uint8_t seq = next_seq();
  k1_ctrl_frame_t frame{};
  k1_build_lift_home_frame(&frame, seq);
  send_frame(frame);
  return wait_command_result(K1_CTRL_TARGET_LIFT, seq, options_.policy.result_timeout);
}

TaskResult Stm32ActuatorClient::lift_clear_fault()
{
  const uint8_t seq = next_seq();
  k1_ctrl_frame_t frame{};
  k1_build_lift_clear_fault_frame(&frame, seq);
  send_frame(frame);
  return wait_command_result(K1_CTRL_TARGET_LIFT, seq, std::chrono::milliseconds(5000));
}

TaskResult Stm32ActuatorClient::lift_move_z(float z_m)
{
  const uint8_t seq = next_seq();
  k1_ctrl_frame_t frame{};
  k1_build_lift_move_frame(&frame, seq, z_m);
  send_frame(frame);
  const auto result = wait_command_result(
    K1_CTRL_TARGET_LIFT, seq, options_.policy.result_timeout);
  if (!result.success) {
    return result;
  }
  const auto status = latest_lift_status();
  if (status && std::fabs(status->current_z - z_m) > options_.policy.lift_z_tolerance_m) {
    std::ostringstream oss;
    oss << "lift reached " << status->current_z << " m, target=" << z_m;
    return make_failure(oss.str());
  }
  return result;
}

std::optional<k1_arm_status_t> Stm32ActuatorClient::latest_arm_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_arm_status_;
}

std::optional<k1_lift_status_t> Stm32ActuatorClient::latest_lift_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_lift_status_;
}

std::optional<k1_chassis_status_t> Stm32ActuatorClient::latest_chassis_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_chassis_status_;
}

std::optional<k1_arm_status_t> Stm32ActuatorClient::wait_for_arm_status(
  std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const uint32_t initial_tick = latest_arm_status_ ? latest_arm_status_->tick_ms : 0U;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (latest_arm_status_ && latest_arm_status_->tick_ms != initial_tick) {
      return latest_arm_status_;
    }
    cv_.wait_until(lock, deadline);
  }
  return latest_arm_status_;
}

std::optional<k1_lift_status_t> Stm32ActuatorClient::wait_for_lift_status(
  std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const uint32_t initial_tick = latest_lift_status_ ? latest_lift_status_->tick_ms : 0U;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (latest_lift_status_ && latest_lift_status_->tick_ms != initial_tick) {
      return latest_lift_status_;
    }
    cv_.wait_until(lock, deadline);
  }
  return latest_lift_status_;
}

void Stm32ActuatorClient::read_loop()
{
  while (running_) {
    const auto data = transport_->read_available(options_.read_timeout);
    for (const uint8_t byte : data) {
      k1_ctrl_frame_t frame{};
      if (k1_ctrl_parser_input(&parser_, byte, &frame) != 0) {
        process_frame(frame);
      }
    }
  }
}

void Stm32ActuatorClient::process_frame(const k1_ctrl_frame_t & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);

  k1_chassis_status_t chassis{};
  if (k1_decode_chassis_status(&frame, &chassis) != 0) {
    latest_chassis_status_ = chassis;
    cv_.notify_all();
    return;
  }

  k1_arm_status_t arm_status{};
  if (k1_decode_arm_status(&frame, &arm_status) != 0) {
    latest_arm_status_ = arm_status;
    cv_.notify_all();
    return;
  }

  k1_lift_status_t lift_status{};
  if (k1_decode_lift_status(&frame, &lift_status) != 0) {
    latest_lift_status_ = lift_status;
    cv_.notify_all();
    return;
  }

  k1_arm_result_t arm_result{};
  if (k1_decode_arm_result(&frame, &arm_result) != 0) {
    ResultEnvelope envelope;
    envelope.target = K1_CTRL_TARGET_ARM;
    envelope.arm = arm_result;
    results_.push_back(envelope);
    cv_.notify_all();
    return;
  }

  k1_lift_result_t lift_result{};
  if (k1_decode_lift_result(&frame, &lift_result) != 0) {
    ResultEnvelope envelope;
    envelope.target = K1_CTRL_TARGET_LIFT;
    envelope.lift = lift_result;
    results_.push_back(envelope);
    cv_.notify_all();
  }
}

TaskResult Stm32ActuatorClient::apply_arm_pose_tolerance(
  const TaskResult & failure, float x, float y, float z)
{
  const auto status = wait_for_arm_status(std::chrono::milliseconds(250));
  if (!status) {
    return failure;
  }
  if (status->has_fault != 0U) {
    return failure;
  }
  const double error = arm_xyz_error(*status, x, y, z);
  if (error <= options_.policy.arm_pose_xyz_tolerance_m) {
    std::ostringstream oss;
    oss << "arm pose assumed OK, xyz_error=" << error << " m";
    return make_success(oss.str());
  }
  return failure;
}

TaskResult Stm32ActuatorClient::apply_gripper_tolerance_or_policy(
  const TaskResult & failure, float angle_rad)
{
  const auto status = wait_for_arm_status(std::chrono::milliseconds(250));
  if (!status) {
    if (options_.policy.assume_gripper_success_without_fault) {
      return make_success("gripper assumed OK without fresh ARM status");
    }
    return failure;
  }
  if (status->has_fault != 0U) {
    return failure;
  }
  const float error = std::fabs(status->current_gripper_rad - angle_rad);
  if (error <= options_.policy.gripper_tolerance_rad ||
      options_.policy.assume_gripper_success_without_fault) {
    std::ostringstream oss;
    oss << "gripper assumed OK, error=" << error << " rad";
    return make_success(oss.str());
  }
  return failure;
}

}  // namespace k1muse_control_manager
