#ifndef _MDL_CONTROL_PROTOCOL_H_
#define _MDL_CONTROL_PROTOCOL_H_

#include <stdint.h>

#define CTRL_FRAME_SOF1 (0xA5U)
#define CTRL_FRAME_SOF2 (0x5AU)
#define CTRL_PROTOCOL_MAX_PAYLOAD (64U)

typedef enum {
    CTRL_SRC_NONE = 0x00,
    CTRL_SRC_BT = 0x01,
    CTRL_SRC_HOST = 0x02,
    CTRL_SRC_MCU = 0x10,
} control_source_e;

typedef enum {
    CTRL_TARGET_SYSTEM = 0x00,
    CTRL_TARGET_CHASSIS = 0x01,
    CTRL_TARGET_ARM = 0x02,
    CTRL_TARGET_LIFT = 0x03,
    CTRL_TARGET_PERIPHERAL = 0x04,
} ctrl_target_e;

typedef enum {
    CTRL_SYS_CMD_RESERVED = 0x00,
    CTRL_SYS_CMD_DEMO_STOP = 0x01,
    CTRL_SYS_CMD_DEMO_RUN = 0x02,
    CTRL_SYS_CMD_DEMO_HOME = 0x03,
    CTRL_SYS_RPT_DEMO_STATUS = 0x80,
    CTRL_SYS_RPT_DEMO_RESULT = 0x81,
} ctrl_system_cmd_e;

typedef enum {
    CTRL_DEMO_VARIANT_AUTO = 0x00,
    CTRL_DEMO_VARIANT_DOWN = 0x01,
    CTRL_DEMO_VARIANT_UP = 0x02,
} ctrl_demo_variant_e;

typedef enum {
    CTRL_RESULT_NONE = 0x00,
    CTRL_RESULT_ACCEPTED = 0x01,
    CTRL_RESULT_REJECTED = 0x02,
    CTRL_RESULT_COMPLETED = 0x03,
    CTRL_RESULT_ABORTED = 0x04,
    CTRL_RESULT_FAILED = 0x05,
    CTRL_RESULT_SUPERSEDED = 0x06,
} ctrl_result_e;

typedef enum {
    CTRL_DEMO_REJECT_NONE = 0x00,
    CTRL_DEMO_REJECT_BAD_LENGTH = 0x01,
    CTRL_DEMO_REJECT_QUEUE_FULL = 0x02,
    CTRL_DEMO_REJECT_BUSY = 0x03,
} ctrl_demo_reject_reason_e;

typedef enum {
    CTRL_CHS_CMD_STOP = 0x00,
    CTRL_CHS_CMD_MOV = 0x01,
    CTRL_CHS_CMD_ODOM = 0x02,
    CTRL_CHS_RPT_STATUS = 0x80,
} ctrl_chassis_cmd_e;

typedef enum {
    CTRL_CHS_STATE_IDLE = 0x00,
    CTRL_CHS_STATE_MOVING = 0x01,
    CTRL_CHS_STATE_TIMEOUT = 0x02,
    CTRL_CHS_STATE_FAULT = 0x03,
} ctrl_chassis_state_e;

typedef enum {
    CTRL_ARM_CMD_STOP = 0x00,
    CTRL_ARM_CMD_HOME = 0x01,
    CTRL_ARM_CMD_MOVE_XYZ = 0x02,
    CTRL_ARM_CMD_MOVE_POSE = 0x03,
    CTRL_ARM_CMD_GRIPPER = 0x04,
    CTRL_ARM_CMD_CLEAR_FAULT = 0x05,
    CTRL_ARM_CMD_DISABLE_TORQUE = 0x06,
    CTRL_ARM_RPT_STATUS = 0x80,
    CTRL_ARM_RPT_RESULT = 0x81,
} ctrl_arm_cmd_e;

typedef enum {
    CTRL_LIFT_CMD_STOP = 0x00,
    CTRL_LIFT_CMD_HOME = 0x01,
    CTRL_LIFT_CMD_MOVE_Z = 0x02,
    CTRL_LIFT_CMD_CLEAR_FAULT = 0x03,
    CTRL_LIFT_RPT_STATUS = 0x80,
    CTRL_LIFT_RPT_RESULT = 0x81,
} ctrl_lift_cmd_e;

typedef enum {
    CTRL_PERIPH_RPT_STATUS = 0x80,
} ctrl_peripheral_cmd_e;

typedef enum {
    CTRL_PERIPH_POWER_NORMAL = 0x00,
    CTRL_PERIPH_POWER_LOW = 0x01,
    CTRL_PERIPH_POWER_CRITICAL = 0x02,
    CTRL_PERIPH_POWER_FAULT = 0x03,
} ctrl_peripheral_power_state_e;

typedef enum {
    CTRL_CHS_MOVE_LCS = 0x00,
    CTRL_CHS_MOVE_WCS = 0x01,
} ctrl_chassis_move_cs_e;

#pragma pack(push, 1)
typedef struct {
    uint8_t move_cs;
    float direction;
    float v;
    float omega;
} ctrl_chassis_mov_payload_t;

typedef struct {
    uint8_t demo_id;
    uint8_t src_layer;
    uint8_t dst_layer;
    uint8_t variant;
} ctrl_demo_run_payload_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t state;
    uint8_t fault;
    uint8_t demo_id;
    uint8_t src_layer;
    uint8_t dst_layer;
    uint8_t variant;
    uint8_t step_index;
    uint8_t step_count;
    uint8_t active;
    uint8_t reserved[3];
} ctrl_demo_status_payload_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t request_seq;
    uint8_t request_cmd;
    uint8_t request_source;
    uint8_t result;
    uint8_t reject_reason;
    uint8_t fault_reason;
    uint8_t state_after;
    uint8_t demo_id;
    uint8_t src_layer;
    uint8_t dst_layer;
    uint8_t variant;
    uint8_t step_index;
    uint8_t step_count;
    uint8_t reserved[3];
} ctrl_demo_result_payload_t;

typedef struct {
    float direction;
    float x;
    float y;
} ctrl_chassis_odom_payload_t;

typedef struct {
    uint32_t tick_ms;

    uint8_t state;
    uint8_t move_cs;
    uint8_t motor_block_flags; /* bit0 LF, bit1 RF, bit2 RB, bit3 LB */
    uint8_t reserved[1];

    float WCS_vx;
    float WCS_vy;
    float omega;

    float WCS_x;
    float WCS_y;
    float WCS_direction;
} ctrl_chassis_status_payload_t;

typedef struct {
    float x;
    float y;
    float z;
} ctrl_arm_move_xyz_payload_t;

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
} ctrl_arm_move_pose_payload_t;

typedef struct {
    float gripper_rad;
} ctrl_arm_gripper_payload_t;

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
} ctrl_arm_status_payload_t;

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
} ctrl_arm_result_payload_t;

typedef struct {
    float z;
} ctrl_lift_z_payload_t;

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

    int32_t motor_total_encoder_count;
} ctrl_lift_status_payload_t;

typedef struct {
    uint32_t tick_ms;

    uint8_t request_seq;
    uint8_t request_cmd;
    uint8_t request_source;
    uint8_t result;

    uint8_t reject_reason;
    uint8_t fault_reason;
    uint8_t state_after;
    uint8_t reserved0;

    float requested_z;
    float accepted_z;
    float current_z;
    float z_min;
    float z_max;
} ctrl_lift_result_payload_t;

typedef struct {
    uint32_t tick_ms;
    uint8_t gas_detected;
    uint8_t power_state;
    uint16_t power_mv;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_pct;
} ctrl_peripheral_status_payload_t;
#pragma pack(pop)

typedef struct {
    uint8_t src;
    uint8_t target;
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[CTRL_PROTOCOL_MAX_PAYLOAD];
} ctrl_frame_t;

typedef char ctrl_arm_status_payload_size_check[(sizeof(ctrl_arm_status_payload_t) == 48U) ? 1 : -1];
typedef char ctrl_arm_result_payload_size_check[(sizeof(ctrl_arm_result_payload_t) == 60U) ? 1 : -1];
typedef char ctrl_lift_status_payload_size_check[(sizeof(ctrl_lift_status_payload_t) == 36U) ? 1 : -1];
typedef char ctrl_lift_result_payload_size_check[(sizeof(ctrl_lift_result_payload_t) == 32U) ? 1 : -1];
typedef char ctrl_peripheral_status_payload_size_check[(sizeof(ctrl_peripheral_status_payload_t) == 12U) ? 1 : -1];
typedef char ctrl_demo_run_payload_size_check[(sizeof(ctrl_demo_run_payload_t) == 4U) ? 1 : -1];
typedef char ctrl_demo_status_payload_size_check[(sizeof(ctrl_demo_status_payload_t) == 16U) ? 1 : -1];
typedef char ctrl_demo_result_payload_size_check[(sizeof(ctrl_demo_result_payload_t) == 20U) ? 1 : -1];

typedef enum {
    CONTROL_PROTOCOL_WAIT_SOF1 = 0,
    CONTROL_PROTOCOL_WAIT_SOF2,
    CONTROL_PROTOCOL_READ_HEADER,
    CONTROL_PROTOCOL_READ_PAYLOAD,
    CONTROL_PROTOCOL_READ_CRC,
} control_protocol_state_e;

typedef struct {
    control_protocol_state_e state;
    ctrl_frame_t frame;
    uint8_t header_buf[5];
    uint8_t header_index;
    uint8_t payload_index;
} control_protocol_t;

void control_protocol_init(control_protocol_t *protocol);
uint8_t control_protocol_input_byte(control_protocol_t *protocol, uint8_t byte,
                                    ctrl_frame_t *frame);
uint8_t control_protocol_input_buffer(control_protocol_t *protocol,
                                      const uint8_t *data, uint16_t len,
                                      ctrl_frame_t *frame);
uint16_t control_protocol_encode_frame(uint8_t src,
                                       uint8_t target,
                                       uint8_t cmd,
                                       uint8_t seq,
                                       const uint8_t *payload,
                                       uint8_t len,
                                       uint8_t *out_buf,
                                       uint16_t out_size);

#endif /* _MDL_CONTROL_PROTOCOL_H_ */
