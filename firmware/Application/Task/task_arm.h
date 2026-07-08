#ifndef TASK_ARM_H
#define TASK_ARM_H

#include "cmsis_os.h"
#include "dev_sts_servo.h"
#include "mdl_so101_arm.h"
#include "mdl_so101_arm_config.h"
#include <stdint.h>

#define ARM_TASK_ACTIVE_JOINT_COUNT (5U)
#define ARM_TASK_FAULT_SERVO_NONE (0xFFU)

typedef enum {
    ARM_TASK_INIT = 0,
    ARM_TASK_ENABLE_TORQUE,
    ARM_TASK_HOME,
    ARM_TASK_IDLE,
    ARM_TASK_EXECUTE,
    ARM_TASK_WAIT_TARGET,
    ARM_TASK_REACHED,
    ARM_TASK_FAULT,
    ARM_TASK_TORQUE_DISABLED,
} arm_task_state_t;

typedef enum {
    ARM_TASK_OK = 0,
    ARM_TASK_BUSY,
    ARM_TASK_ERR_BUS,
    ARM_TASK_ERR_FAULT,
    ARM_TASK_ERR_IK,
    ARM_TASK_ERR_TIMEOUT,
    ARM_TASK_ERR_PARAM,
} arm_task_status_t;

typedef enum {
    ARM_RESULT_NONE = 0,
    ARM_RESULT_ACCEPTED,
    ARM_RESULT_REJECTED,
    ARM_RESULT_COMPLETED,
    ARM_RESULT_ABORTED,
    ARM_RESULT_FAILED,
    ARM_RESULT_SUPERSEDED,
} arm_result_t;

typedef enum {
    ARM_REJECT_NONE = 0,
    ARM_REJECT_BUSY,
    ARM_REJECT_OWNER,
    ARM_REJECT_IN_FAULT,
    ARM_REJECT_TORQUE_DISABLED,
    ARM_REJECT_PARAM,
    ARM_REJECT_TARGET_OUT_OF_RANGE,
    ARM_REJECT_IK_UNREACHABLE,
    ARM_REJECT_UNKNOWN_CMD,
} arm_reject_reason_t;

typedef enum {
    ARM_FAULT_SRC_NONE = 0,
    ARM_FAULT_SRC_TASK = 1,
    ARM_FAULT_SRC_MDL = 2,
    ARM_FAULT_SRC_SERVO_BUS = 3,
    ARM_FAULT_SRC_SERVO = 4,
    ARM_FAULT_SRC_IK = 5,
    ARM_FAULT_SRC_TIMEOUT = 6,
} arm_fault_source_t;

typedef enum {
    ARM_DIAG_OK = 0,
    ARM_DIAG_BUSY = 1,
    ARM_DIAG_PARAM_ERROR = 2,
    ARM_DIAG_IK_UNREACHABLE = 3,
    ARM_DIAG_SERVO_FAULT = 4,
    ARM_DIAG_SERVO_BUS_TIMEOUT = 5,
    ARM_DIAG_MDL_FAULT = 6,
    ARM_DIAG_TASK_TIMEOUT = 7,
} arm_diag_code_t;

typedef enum {
    ARM_DIAG_SEV_INFO = 0,
    ARM_DIAG_SEV_WARN,
    ARM_DIAG_SEV_ERROR,
    ARM_DIAG_SEV_FATAL,
} arm_diag_severity_t;

/* Fast fault locator for Watch and future uplink replies. */
typedef struct {
    uint32_t tick_ms;
    arm_diag_code_t code;
    arm_diag_severity_t severity;
    arm_fault_source_t fault_source;
    arm_task_state_t task_state;
    arm_task_status_t task_status;
    uint8_t fault_servo_index;
    uint32_t detail_u32;
    float detail_f32;
} arm_diag_summary_t;

/* Periodic lightweight state for upper-computer display and scheduling. */
typedef struct {
    uint32_t tick_ms;
    arm_task_state_t task_state;
    arm_task_status_t task_status;
    float current_xyz[3];
    float target_xyz[3];
    float current_joint_rad[SO101_ACTIVE_JOINT_COUNT];
    float target_joint_rad[SO101_ACTIVE_JOINT_COUNT];
    float gripper_rad;
    uint8_t is_reached;
    uint8_t is_busy;
    uint8_t has_fault;
} arm_runtime_state_t;

/* Low-frequency detail: expand only after summary points to arm internals. */
typedef struct {
    so101_arm_status_t last_mdl_status;
    robot_status_t last_robot_status;
    robot_status_t ik_status;
    uint16_t ik_iterations;
    float ik_position_error_m;
    float ik_pitch_error_rad;
    float ik_joint_limit_margin_rad;
    float ik_side_error_m;
    uint32_t fault_flags;
    uint32_t fatal_fault_flags;
    uint32_t consecutive_timeout_count;
    uint32_t over_temp_elapsed_ms;
    uint32_t voltage_fault_elapsed_ms;
    uint32_t bus_timeout_elapsed_ms;
    uint32_t status_error_elapsed_ms;
    uint32_t servo_fault_flags[SO101_ARM_SERVO_COUNT];
    sts_servo_status_t servo_last_status[SO101_ARM_SERVO_COUNT];
    uint8_t servo_voltage[SO101_ARM_SERVO_COUNT];
    uint8_t servo_temperature[SO101_ARM_SERVO_COUNT];
    uint16_t servo_load[SO101_ARM_SERVO_COUNT];
    uint16_t servo_current[SO101_ARM_SERVO_COUNT];

    sts_servo_status_t bus_last_status;
    sts_servo_parser_state_t bus_parser_state;
    uint8_t bus_tx_busy;
    uint8_t bus_queue_count;
    uint8_t bus_response_count;
    uint8_t bus_pending_response_count;
} arm_detail_state_t;

typedef struct {
    arm_diag_summary_t summary;
    arm_runtime_state_t runtime;
    arm_detail_state_t detail;
} arm_state_snapshot_t;

typedef enum {
    ARM_CMD_STOP = 0x00,
    ARM_CMD_HOME = 0x01,
    ARM_CMD_MOVE_XYZ = 0x02,
    ARM_CMD_MOVE_POSE = 0x03,
    ARM_CMD_GRIPPER = 0x04,
    ARM_CMD_CLEAR_FAULT = 0x05,
    ARM_CMD_DISABLE_TORQUE = 0x06,
} arm_cmd_type_e;

typedef struct {
    uint8_t type;
    uint8_t source;
    uint8_t seq;
    uint8_t reserved;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float gripper_rad;
    uint32_t tick;
} arm_cmd_msg_t;

typedef char arm_cmd_msg_size_check[(sizeof(arm_cmd_msg_t) == 32U) ? 1 : -1];

extern osMessageQueueId_t ArmCmdQueueHandle;
extern volatile arm_state_snapshot_t g_arm_state;

arm_task_state_t arm_task_get_state(void);
arm_task_status_t arm_task_get_status(void);
uint8_t arm_task_is_ready(void);

#endif /* TASK_ARM_H */
