#ifndef K1MUSE_CONTROL_CORE__ACTUATOR_PROTOCOL_H_
#define K1MUSE_CONTROL_CORE__ACTUATOR_PROTOCOL_H_

#include <stdint.h>

#include "k1muse_control_core/control_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    K1_CTRL_SRC_NONE = 0x00,
    K1_CTRL_SRC_BT = 0x01,
    K1_CTRL_SRC_HOST = 0x02,
    K1_CTRL_SRC_MCU = 0x10,
} k1_ctrl_source_t;

typedef enum {
    K1_CTRL_TARGET_SYSTEM = 0x00,
    K1_CTRL_TARGET_CHASSIS = 0x01,
    K1_CTRL_TARGET_ARM = 0x02,
    K1_CTRL_TARGET_LIFT = 0x03,
} k1_ctrl_target_t;

typedef enum {
    K1_CHASSIS_CMD_STOP = 0x00,
    K1_CHASSIS_CMD_MOV = 0x01,
    K1_CHASSIS_CMD_ODOM = 0x02,
    K1_CHASSIS_RPT_STATUS = 0x80,
} k1_chassis_cmd_t;

typedef enum {
    K1_ARM_CMD_STOP = 0x00,
    K1_ARM_CMD_HOME = 0x01,
    K1_ARM_CMD_MOVE_XYZ = 0x02,
    K1_ARM_CMD_MOVE_POSE = 0x03,
    K1_ARM_CMD_GRIPPER = 0x04,
    K1_ARM_CMD_CLEAR_FAULT = 0x05,
    K1_ARM_CMD_DISABLE_TORQUE = 0x06,
    K1_ARM_RPT_STATUS = 0x80,
    K1_ARM_RPT_RESULT = 0x81,
} k1_arm_cmd_t;

typedef enum {
    K1_LIFT_CMD_STOP = 0x00,
    K1_LIFT_CMD_HOME = 0x01,
    K1_LIFT_CMD_MOVE_Z = 0x02,
    K1_LIFT_CMD_CLEAR_FAULT = 0x03,
    K1_LIFT_RPT_STATUS = 0x80,
    K1_LIFT_RPT_RESULT = 0x81,
} k1_lift_cmd_t;

typedef enum {
    K1_RESULT_NONE = 0,
    K1_RESULT_ACCEPTED = 1,
    K1_RESULT_REJECTED = 2,
    K1_RESULT_COMPLETED = 3,
    K1_RESULT_ABORTED = 4,
    K1_RESULT_FAILED = 5,
    K1_RESULT_SUPERSEDED = 6,
} k1_command_result_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t state;
    uint8_t move_cs;
    uint8_t motor_block_flags;
    float vx;
    float vy;
    float omega;
    float x;
    float y;
    float direction;
} k1_chassis_status_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t state;
    uint8_t status;
    uint8_t is_busy;
    uint8_t has_fault;
    uint8_t active_cmd;
    uint8_t active_source;
    uint8_t active_seq;
    uint8_t diag_code;
    float current_x;
    float current_y;
    float current_z;
    float target_x;
    float target_y;
    float target_z;
    float current_gripper_rad;
    float target_gripper_rad;
    uint32_t fault_flags;
} k1_arm_status_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t request_seq;
    uint8_t request_cmd;
    uint8_t request_source;
    uint8_t result;
    uint8_t reject_reason;
    uint8_t fault_source;
    uint8_t state_after;
    uint8_t diag_code;
    float requested_x;
    float requested_y;
    float requested_z;
    float requested_gripper_rad;
    float accepted_x;
    float accepted_y;
    float accepted_z;
    float accepted_gripper_rad;
    float current_x;
    float current_y;
    float current_z;
    float current_gripper_rad;
} k1_arm_result_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t state;
    uint8_t home_state;
    uint8_t is_homed;
    uint8_t is_busy;
    uint8_t has_fault;
    uint8_t fault_reason;
    uint8_t motor_blocked;
    uint8_t home_sensor_level;
    float current_z;
    float target_z;
    float current_v;
    float target_v;
    float position_error;
    int32_t motor_encoder_count;
} k1_lift_status_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t request_seq;
    uint8_t request_cmd;
    uint8_t request_source;
    uint8_t result;
    uint8_t reject_reason;
    uint8_t fault_reason;
    uint8_t state_after;
    float requested_z;
    float accepted_z;
    float current_z;
    float z_min;
    float z_max;
} k1_lift_result_t;

void k1_build_stop_frame(k1_ctrl_frame_t *frame, uint8_t target, uint8_t seq);
void k1_build_chassis_move_frame(k1_ctrl_frame_t *frame, uint8_t seq,
                                 uint8_t move_cs, float direction,
                                 float velocity, float omega);
void k1_build_lift_move_frame(k1_ctrl_frame_t *frame, uint8_t seq, float z);
void k1_build_lift_home_frame(k1_ctrl_frame_t *frame, uint8_t seq);
void k1_build_arm_home_frame(k1_ctrl_frame_t *frame, uint8_t seq);
void k1_build_arm_pose_frame(k1_ctrl_frame_t *frame, uint8_t seq,
                             float x, float y, float z,
                             float roll, float pitch);
void k1_build_arm_gripper_frame(k1_ctrl_frame_t *frame, uint8_t seq, float angle);
void k1_build_arm_clear_fault_frame(k1_ctrl_frame_t *frame, uint8_t seq);
void k1_build_lift_clear_fault_frame(k1_ctrl_frame_t *frame, uint8_t seq);

int k1_decode_chassis_status(const k1_ctrl_frame_t *frame, k1_chassis_status_t *status);
int k1_decode_arm_status(const k1_ctrl_frame_t *frame, k1_arm_status_t *status);
int k1_decode_arm_result(const k1_ctrl_frame_t *frame, k1_arm_result_t *result);
int k1_decode_lift_status(const k1_ctrl_frame_t *frame, k1_lift_status_t *status);
int k1_decode_lift_result(const k1_ctrl_frame_t *frame, k1_lift_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
