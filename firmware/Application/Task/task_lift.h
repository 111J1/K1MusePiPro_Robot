#ifndef _TASK_LIFT_H_
#define _TASK_LIFT_H_

#include "cmsis_os.h"
#include "mdl_lift.h"
#include <stdint.h>

typedef enum {
    LIFT_CMD_STOP = 0x00,
    LIFT_CMD_HOME = 0x01,
    LIFT_CMD_MOVE_Z = 0x02,
    LIFT_CMD_CLEAR_FAULT = 0x03,
} lift_cmd_type_e;

typedef struct {
    uint8_t type;
    uint8_t source;
    uint8_t seq;
    uint8_t reserved;
    float z;
    uint32_t tick;
} lift_cmd_msg_t;

typedef enum {
    LIFT_FAULT_NONE = 0,
    LIFT_FAULT_HOME_SENSOR_NOT_INIT,
    LIFT_FAULT_HOME_TIMEOUT,
    LIFT_FAULT_MOTOR_BLOCKED,
    LIFT_FAULT_CONTROL_TIMEOUT,
    LIFT_FAULT_SENSOR_POSITION_MISMATCH,
} lift_fault_reason_t;

typedef enum {
    LIFT_REJECT_NONE = 0,
    LIFT_REJECT_NOT_HOMED,
    LIFT_REJECT_BUSY,
    LIFT_REJECT_TARGET_OUT_OF_RANGE,
    LIFT_REJECT_IN_FAULT,
    LIFT_REJECT_UNKNOWN_CMD,
} lift_reject_reason_t;

typedef enum {
    LIFT_RESULT_NONE = 0,
    LIFT_RESULT_ACCEPTED,
    LIFT_RESULT_REJECTED,
    LIFT_RESULT_COMPLETED,
    LIFT_RESULT_ABORTED,
    LIFT_RESULT_FAILED,
    LIFT_RESULT_SUPERSEDED,
} lift_result_t;

typedef enum {
    LIFT_DIAG_OK = 0,
    LIFT_DIAG_BUSY,
    LIFT_DIAG_NOT_HOMED,
    LIFT_DIAG_HOME_TIMEOUT,
    LIFT_DIAG_HOME_SENSOR_ERROR,
    LIFT_DIAG_MOTOR_BLOCKED,
    LIFT_DIAG_CONTROL_TIMEOUT,
    LIFT_DIAG_POSITION_MISMATCH,
} lift_diag_code_t;

typedef enum {
    LIFT_DIAG_SEV_INFO = 0,
    LIFT_DIAG_SEV_WARN,
    LIFT_DIAG_SEV_ERROR,
    LIFT_DIAG_SEV_FATAL,
} lift_diag_severity_t;

typedef struct {
    uint32_t tick_ms;
    lift_diag_code_t code;
    lift_diag_severity_t severity;
    lift_fault_reason_t fault_reason;
    lift_state_e state;
    lift_home_state_e home_state;
    uint32_t detail_u32;
    float detail_f32;
} lift_diag_summary_t;

typedef struct {
    uint32_t tick_ms;
    lift_state_e state;
    lift_home_state_e home_state;
    uint8_t is_homed;
    uint8_t is_busy;
    uint8_t has_fault;
    float current_z;
    float target_z;
    float current_v;
    float target_v;
    float position_error;
    int32_t motor_total_encoder_count;
} lift_runtime_state_t;

typedef struct {
    lift_fault_reason_t fault_reason;
    lift_reject_reason_t reject_reason;
    float rejected_target_z;
    float reject_z_min;
    float reject_z_max;

    uint8_t home_start_level;
    uint8_t bottom_sensor_initialized;
    uint8_t bottom_sensor_level;
    uint8_t bottom_sensor_last_level;
    uint8_t bottom_sensor_rising_edge;
    uint8_t bottom_sensor_falling_edge;

    uint8_t middle_sensor_initialized;
    uint8_t middle_sensor_level;
    uint8_t middle_sensor_last_level;
    uint8_t middle_sensor_rising_edge;
    uint8_t middle_sensor_falling_edge;

    uint8_t top_sensor_initialized;
    uint8_t top_sensor_level;
    uint8_t top_sensor_last_level;
    uint8_t top_sensor_rising_edge;
    uint8_t top_sensor_falling_edge;

    uint8_t last_ref_sensor;
    uint8_t last_ref_edge;
    float last_ref_z;
    float last_ref_error;

    float z_min;
    float z_max;
    float max_v;
    float reached_eps;
    float position_pid_out;

    float motor_current_rpm;
    float motor_target_rpm;
    int8_t motor_current_direction;
    int8_t motor_target_direction;
    int32_t motor_total_encoder_count;
    uint8_t motor_initialized;
    uint8_t motor_blocked;
    uint16_t motor_blocked_cnt;
    uint16_t motor_block_max_cnt;
    float motor_pid_out;

    uint32_t home_start_tick;
    uint32_t home_elapsed_tick;
} lift_detail_state_t;

typedef struct {
    lift_diag_summary_t summary;
    lift_runtime_state_t runtime;
    lift_detail_state_t detail;
} lift_state_snapshot_t;

extern osMessageQueueId_t LiftCmdQueueHandle;
extern volatile lift_state_snapshot_t g_lift_state;

void StartLiftTask(void *argument);

#endif /* _TASK_LIFT_H_ */
