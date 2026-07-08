#include "k1muse_control_core/actuator_protocol.h"

#include <string.h>

static void init_host_frame(k1_ctrl_frame_t *frame, uint8_t target,
                            uint8_t cmd, uint8_t seq, uint8_t len)
{
    memset(frame, 0, sizeof(*frame));
    frame->src = K1_CTRL_SRC_HOST;
    frame->target = target;
    frame->cmd = cmd;
    frame->seq = seq;
    frame->len = len;
}

void k1_build_stop_frame(k1_ctrl_frame_t *frame, uint8_t target, uint8_t seq)
{
    if (frame != NULL) {
        init_host_frame(frame, target, 0x00U, seq, 0U);
    }
}

void k1_build_chassis_move_frame(k1_ctrl_frame_t *frame, uint8_t seq,
                                 uint8_t move_cs, float direction,
                                 float velocity, float omega)
{
    if (frame == NULL) {
        return;
    }
    init_host_frame(frame, K1_CTRL_TARGET_CHASSIS, K1_CHASSIS_CMD_MOV, seq, 13U);
    frame->payload[0] = move_cs;
    k1_put_f32_le(&frame->payload[1], direction);
    k1_put_f32_le(&frame->payload[5], velocity);
    k1_put_f32_le(&frame->payload[9], omega);
}

void k1_build_lift_move_frame(k1_ctrl_frame_t *frame, uint8_t seq, float z)
{
    if (frame == NULL) {
        return;
    }
    init_host_frame(frame, K1_CTRL_TARGET_LIFT, K1_LIFT_CMD_MOVE_Z, seq, 4U);
    k1_put_f32_le(frame->payload, z);
}

void k1_build_lift_home_frame(k1_ctrl_frame_t *frame, uint8_t seq)
{
    if (frame != NULL) {
        init_host_frame(frame, K1_CTRL_TARGET_LIFT, K1_LIFT_CMD_HOME, seq, 0U);
    }
}

void k1_build_arm_home_frame(k1_ctrl_frame_t *frame, uint8_t seq)
{
    if (frame != NULL) {
        init_host_frame(frame, K1_CTRL_TARGET_ARM, K1_ARM_CMD_HOME, seq, 0U);
    }
}

void k1_build_arm_pose_frame(k1_ctrl_frame_t *frame, uint8_t seq,
                             float x, float y, float z,
                             float roll, float pitch)
{
    if (frame == NULL) {
        return;
    }
    init_host_frame(frame, K1_CTRL_TARGET_ARM, K1_ARM_CMD_MOVE_POSE, seq, 20U);
    k1_put_f32_le(&frame->payload[0], x);
    k1_put_f32_le(&frame->payload[4], y);
    k1_put_f32_le(&frame->payload[8], z);
    k1_put_f32_le(&frame->payload[12], roll);
    k1_put_f32_le(&frame->payload[16], pitch);
}

void k1_build_arm_gripper_frame(k1_ctrl_frame_t *frame, uint8_t seq, float angle)
{
    if (frame == NULL) {
        return;
    }
    init_host_frame(frame, K1_CTRL_TARGET_ARM, K1_ARM_CMD_GRIPPER, seq, 4U);
    k1_put_f32_le(frame->payload, angle);
}

void k1_build_arm_clear_fault_frame(k1_ctrl_frame_t *frame, uint8_t seq)
{
    if (frame != NULL) {
        init_host_frame(frame, K1_CTRL_TARGET_ARM, K1_ARM_CMD_CLEAR_FAULT, seq, 0U);
    }
}

void k1_build_lift_clear_fault_frame(k1_ctrl_frame_t *frame, uint8_t seq)
{
    if (frame != NULL) {
        init_host_frame(frame, K1_CTRL_TARGET_LIFT, K1_LIFT_CMD_CLEAR_FAULT, seq, 0U);
    }
}

int k1_decode_chassis_status(const k1_ctrl_frame_t *frame, k1_chassis_status_t *status)
{
    if ((frame == NULL) || (status == NULL) ||
        (frame->src != K1_CTRL_SRC_MCU) ||
        (frame->target != K1_CTRL_TARGET_CHASSIS) ||
        (frame->cmd != K1_CHASSIS_RPT_STATUS) || (frame->len != 32U)) {
        return 0;
    }
    status->tick_ms = k1_get_u32_le(&frame->payload[0]);
    status->state = frame->payload[4];
    status->move_cs = frame->payload[5];
    status->motor_block_flags = frame->payload[6];
    status->vx = k1_get_f32_le(&frame->payload[8]);
    status->vy = k1_get_f32_le(&frame->payload[12]);
    status->omega = k1_get_f32_le(&frame->payload[16]);
    status->x = k1_get_f32_le(&frame->payload[20]);
    status->y = k1_get_f32_le(&frame->payload[24]);
    status->direction = k1_get_f32_le(&frame->payload[28]);
    return 1;
}

int k1_decode_arm_status(const k1_ctrl_frame_t *frame, k1_arm_status_t *status)
{
    if ((frame == NULL) || (status == NULL) ||
        (frame->src != K1_CTRL_SRC_MCU) ||
        (frame->target != K1_CTRL_TARGET_ARM) ||
        (frame->cmd != K1_ARM_RPT_STATUS) || (frame->len != 48U)) {
        return 0;
    }
    status->tick_ms = k1_get_u32_le(&frame->payload[0]);
    status->state = frame->payload[4];
    status->status = frame->payload[5];
    status->is_busy = frame->payload[6];
    status->has_fault = frame->payload[7];
    status->active_cmd = frame->payload[8];
    status->active_source = frame->payload[9];
    status->active_seq = frame->payload[10];
    status->diag_code = frame->payload[11];
    status->current_x = k1_get_f32_le(&frame->payload[12]);
    status->current_y = k1_get_f32_le(&frame->payload[16]);
    status->current_z = k1_get_f32_le(&frame->payload[20]);
    status->target_x = k1_get_f32_le(&frame->payload[24]);
    status->target_y = k1_get_f32_le(&frame->payload[28]);
    status->target_z = k1_get_f32_le(&frame->payload[32]);
    status->current_gripper_rad = k1_get_f32_le(&frame->payload[36]);
    status->target_gripper_rad = k1_get_f32_le(&frame->payload[40]);
    status->fault_flags = k1_get_u32_le(&frame->payload[44]);
    return 1;
}

int k1_decode_arm_result(const k1_ctrl_frame_t *frame, k1_arm_result_t *result)
{
    if ((frame == NULL) || (result == NULL) ||
        (frame->src != K1_CTRL_SRC_MCU) ||
        (frame->target != K1_CTRL_TARGET_ARM) ||
        (frame->cmd != K1_ARM_RPT_RESULT) || (frame->len != 60U)) {
        return 0;
    }
    result->tick_ms = k1_get_u32_le(&frame->payload[0]);
    result->request_seq = frame->payload[4];
    result->request_cmd = frame->payload[5];
    result->request_source = frame->payload[6];
    result->result = frame->payload[7];
    result->reject_reason = frame->payload[8];
    result->fault_source = frame->payload[9];
    result->state_after = frame->payload[10];
    result->diag_code = frame->payload[11];
    result->requested_x = k1_get_f32_le(&frame->payload[12]);
    result->requested_y = k1_get_f32_le(&frame->payload[16]);
    result->requested_z = k1_get_f32_le(&frame->payload[20]);
    result->requested_gripper_rad = k1_get_f32_le(&frame->payload[24]);
    result->accepted_x = k1_get_f32_le(&frame->payload[28]);
    result->accepted_y = k1_get_f32_le(&frame->payload[32]);
    result->accepted_z = k1_get_f32_le(&frame->payload[36]);
    result->accepted_gripper_rad = k1_get_f32_le(&frame->payload[40]);
    result->current_x = k1_get_f32_le(&frame->payload[44]);
    result->current_y = k1_get_f32_le(&frame->payload[48]);
    result->current_z = k1_get_f32_le(&frame->payload[52]);
    result->current_gripper_rad = k1_get_f32_le(&frame->payload[56]);
    return 1;
}

int k1_decode_lift_status(const k1_ctrl_frame_t *frame, k1_lift_status_t *status)
{
    if ((frame == NULL) || (status == NULL) ||
        (frame->src != K1_CTRL_SRC_MCU) ||
        (frame->target != K1_CTRL_TARGET_LIFT) ||
        (frame->cmd != K1_LIFT_RPT_STATUS) || (frame->len != 36U)) {
        return 0;
    }
    status->tick_ms = k1_get_u32_le(&frame->payload[0]);
    status->state = frame->payload[4];
    status->home_state = frame->payload[5];
    status->is_homed = frame->payload[6];
    status->is_busy = frame->payload[7];
    status->has_fault = frame->payload[8];
    status->fault_reason = frame->payload[9];
    status->motor_blocked = frame->payload[10];
    status->home_sensor_level = frame->payload[11];
    status->current_z = k1_get_f32_le(&frame->payload[12]);
    status->target_z = k1_get_f32_le(&frame->payload[16]);
    status->current_v = k1_get_f32_le(&frame->payload[20]);
    status->target_v = k1_get_f32_le(&frame->payload[24]);
    status->position_error = k1_get_f32_le(&frame->payload[28]);
    status->motor_encoder_count = (int32_t)k1_get_u32_le(&frame->payload[32]);
    return 1;
}

int k1_decode_lift_result(const k1_ctrl_frame_t *frame, k1_lift_result_t *result)
{
    if ((frame == NULL) || (result == NULL) ||
        (frame->src != K1_CTRL_SRC_MCU) ||
        (frame->target != K1_CTRL_TARGET_LIFT) ||
        (frame->cmd != K1_LIFT_RPT_RESULT) || (frame->len != 32U)) {
        return 0;
    }
    result->tick_ms = k1_get_u32_le(&frame->payload[0]);
    result->request_seq = frame->payload[4];
    result->request_cmd = frame->payload[5];
    result->request_source = frame->payload[6];
    result->result = frame->payload[7];
    result->reject_reason = frame->payload[8];
    result->fault_reason = frame->payload[9];
    result->state_after = frame->payload[10];
    result->requested_z = k1_get_f32_le(&frame->payload[12]);
    result->accepted_z = k1_get_f32_le(&frame->payload[16]);
    result->current_z = k1_get_f32_le(&frame->payload[20]);
    result->z_min = k1_get_f32_le(&frame->payload[24]);
    result->z_max = k1_get_f32_le(&frame->payload[28]);
    return 1;
}
