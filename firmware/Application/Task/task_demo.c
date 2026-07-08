#include "task_demo.h"

#include "demo_macro_config.h"
#include "mdl_control_protocol.h"
#include "task_arm.h"
#include "task_chassis.h"
#include "task_lift.h"
#include "task_telemetry.h"

#define DEMO_TASK_PERIOD_MS (10U)
#define DEMO_CMD_QUEUE_WAIT (0U)
#define DEMO_LAYER_SELECT_SRC (0U)
#define DEMO_LAYER_SELECT_DST (1U)

typedef enum {
    DEMO_STEP_END = 0,
    DEMO_STEP_STOP_CHASSIS,
    DEMO_STEP_CHASSIS_MOV,
    DEMO_STEP_ARM_HOME,
    DEMO_STEP_ARM_POSE,
    DEMO_STEP_ARM_GRIPPER,
    DEMO_STEP_LIFT_HOME_IF_NEEDED,
    DEMO_STEP_LIFT_LAYER,
    DEMO_STEP_LIFT_Z,
    DEMO_STEP_DELAY,
    DEMO_STEP_CHASSIS_MOV_ARC,
} demo_step_type_e;

typedef enum {
    DEMO_POSE_NONE = 0,
    DEMO_POSE_TOP_SAFE,
    DEMO_POSE_TOP_PICK_HOVER,
    DEMO_POSE_TOP_PICK_APPROACH,
    DEMO_POSE_TOP_PICK_DOWN,
    DEMO_POSE_TOP_PICK_LIFT,
    DEMO_POSE_TOP_PLACE_APPROACH,
    DEMO_POSE_TOP_PLACE_DOWN,
    DEMO_POSE_TOP_PLACE_LIFT,
    DEMO_POSE_SIDE_SAFE,
    DEMO_POSE_SIDE_PICK_APPROACH,
    DEMO_POSE_SIDE_PICK_CLEARANCE,
    DEMO_POSE_SIDE_PICK_PRE_INSERT,
    DEMO_POSE_SIDE_PICK_INSERT,
    DEMO_POSE_SIDE_PICK_LIFT,
    DEMO_POSE_SIDE_CARRY_SAFE,
    DEMO_POSE_SIDE_PLACE_APPROACH,
    DEMO_POSE_SIDE_PLACE_CLEARANCE,
    DEMO_POSE_SIDE_PLACE_PRE_INSERT,
    DEMO_POSE_SIDE_PLACE_INSERT,
    DEMO_POSE_SIDE_RETREAT_SAFE,
    DEMO_POSE_KEY_GRIP,
    DEMO_POSE_KEY_TURN,
} demo_pose_id_e;

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
} demo_pose_t;

typedef struct {
    demo_step_type_e type;
    uint8_t arg;
    float value;
    uint32_t timeout_ms;
} demo_step_t;

typedef struct {
    demo_cmd_msg_t request;
    const demo_step_t *steps;
    uint8_t step_count;
    uint8_t step_index;
    uint8_t step_sent;
    uint8_t internal_seq;
    uint32_t step_start_tick;
    uint32_t state_start_tick;
    uint32_t status_last_tick;
    uint8_t terminal_result_sent;
    demo_state_e state;
    demo_fault_e fault;
} demo_context_t;

volatile demo_state_snapshot_t g_demo_state;

static demo_context_t s_demo;

static const demo_step_t s_demo_home_steps[] = {
    {DEMO_STEP_STOP_CHASSIS, 0U, 0.0f, DEMO_STEP_TIMEOUT_CHASSIS_MS},
    {DEMO_STEP_ARM_HOME, 0U, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_LIFT_HOME_IF_NEEDED, 0U, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
};

static const demo_step_t s_demo_static_top_steps[] = {
    {DEMO_STEP_STOP_CHASSIS, 0U, 0.0f, DEMO_STEP_TIMEOUT_CHASSIS_MS},
    {DEMO_STEP_LIFT_HOME_IF_NEEDED, 0U, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_LIFT_Z, 0U, DEMO_TOP_STATIC_LIFT_Z_M, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_OPEN_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PICK_HOVER, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PICK_DOWN, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_CLOSE_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY, 0U, 0.0f, DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PICK_LIFT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_CHASSIS_MOV, 1U, DEMO_CHASSIS_MOVE_SPEED_MPS, DEMO_CHASSIS_MOVE_LEFT_DURATION_MS + DEMO_CHASSIS_MOVE_SETTLE_MS},
    {DEMO_STEP_STOP_CHASSIS, 0U, 0.0f, DEMO_CHASSIS_MOVE_SETTLE_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PLACE_APPROACH, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PLACE_DOWN, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_OPEN_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY, 0U, 0.0f, DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PLACE_LIFT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_HOME, 0U, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
};

static const demo_step_t s_demo_layer_pick_steps[] = {
    {DEMO_STEP_STOP_CHASSIS, 0U, 0.0f, DEMO_STEP_TIMEOUT_CHASSIS_MS},
    {DEMO_STEP_LIFT_HOME_IF_NEEDED, 0U, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_LIFT_LAYER, DEMO_LAYER_SELECT_SRC, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_OPEN_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_APPROACH, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_CLEARANCE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_CLOSE_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY, 0U, 0.0f, DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_PRE_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_CARRY_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
};

static const demo_step_t s_demo_layer_place_steps[] = {
    {DEMO_STEP_STOP_CHASSIS, 0U, 0.0f, DEMO_STEP_TIMEOUT_CHASSIS_MS},
    {DEMO_STEP_LIFT_HOME_IF_NEEDED, 0U, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_LIFT_LAYER, DEMO_LAYER_SELECT_DST, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_CLEARANCE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_PRE_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_OPEN_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY, 0U, 0.0f, DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_APPROACH, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_RETREAT_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_HOME, 0U, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
};

static const demo_step_t s_demo_layer_transfer_steps[] = {
    {DEMO_STEP_STOP_CHASSIS, 0U, 0.0f, DEMO_STEP_TIMEOUT_CHASSIS_MS},
    {DEMO_STEP_LIFT_HOME_IF_NEEDED, 0U, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_LIFT_LAYER, DEMO_LAYER_SELECT_SRC, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_OPEN_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_APPROACH, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_CLEARANCE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_CLOSE_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY, 0U, 0.0f, DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PICK_PRE_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_CARRY_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_LIFT_LAYER, DEMO_LAYER_SELECT_DST, 0.0f, DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_CLEARANCE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_PRE_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_INSERT, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER, 0U, DEMO_GRIPPER_OPEN_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY, 0U, 0.0f, DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_PLACE_APPROACH, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_SIDE_RETREAT_SAFE, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_HOME, 0U, 0.0f, DEMO_STEP_TIMEOUT_ARM_MS},
};

static const demo_step_t s_demo_key_turn_steps[] = {
    {DEMO_STEP_STOP_CHASSIS,         0U, 0.0f,                   DEMO_STEP_TIMEOUT_CHASSIS_MS},
    {DEMO_STEP_LIFT_HOME_IF_NEEDED,  0U, 0.0f,                   DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_LIFT_Z,               0U, DEMO_KEY_LIFT_Z_M,      DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_GRIPPER,          0U, DEMO_GRIPPER_OPEN_RAD,  DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE,  DEMO_POSE_KEY_GRIP,  0.0f,             DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER,          0U, DEMO_GRIPPER_CLOSE_RAD, DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY,                0U, 0.0f,                   DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_ARM_POSE,  DEMO_POSE_KEY_TURN,  0.0f,             DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_CHASSIS_MOV_ARC,      4U, DEMO_KEY_PULL_SPEED_MPS, DEMO_KEY_PULL_DURATION_MS},
    {DEMO_STEP_ARM_GRIPPER,          0U, DEMO_GRIPPER_OPEN_RAD,  DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_HOME,             0U, 0.0f,                   DEMO_STEP_TIMEOUT_ARM_MS},
};

static const demo_step_t s_demo_cabinet_pull_steps[] = {
    {DEMO_STEP_STOP_CHASSIS,        0U, 0.0f,                       DEMO_STEP_TIMEOUT_CHASSIS_MS},
    {DEMO_STEP_LIFT_HOME_IF_NEEDED, 0U, 0.0f,                       DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_LIFT_Z,              0U, DEMO_CABINET_LIFT_Z_M,      DEMO_STEP_TIMEOUT_LIFT_MS},
    {DEMO_STEP_ARM_GRIPPER,         0U, DEMO_GRIPPER_OPEN_RAD,      DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_SAFE,          0.0f,         DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PICK_APPROACH, 0.0f,         DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PICK_DOWN,     0.0f,         DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_GRIPPER,         0U, DEMO_GRIPPER_CLOSE_RAD,     DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_DELAY,               0U, 0.0f,                       DEMO_GRIPPER_SETTLE_MS},
    {DEMO_STEP_CHASSIS_MOV,         2U, DEMO_CABINET_PULL_SPEED_MPS, DEMO_CABINET_PULL_DURATION_MS + DEMO_CHASSIS_MOVE_SETTLE_MS},
    {DEMO_STEP_STOP_CHASSIS,        0U, 0.0f,                       DEMO_CHASSIS_MOVE_SETTLE_MS},
    {DEMO_STEP_ARM_GRIPPER,         0U, DEMO_GRIPPER_OPEN_RAD,      DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_PICK_LIFT,    0.0f,          DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_POSE, DEMO_POSE_TOP_SAFE,          0.0f,         DEMO_STEP_TIMEOUT_ARM_MS},
    {DEMO_STEP_ARM_HOME,            0U, 0.0f,                       DEMO_STEP_TIMEOUT_ARM_MS},
};

static uint32_t demo_ms_to_ticks(uint32_t ms)
{
    uint32_t freq = osKernelGetTickFreq();

    if (freq == 0U) {
        return ms;
    }
    return (uint32_t)(((uint64_t)ms * (uint64_t)freq + 999ULL) / 1000ULL);
}

static uint32_t demo_tick_to_ms(uint32_t tick)
{
    uint32_t freq = osKernelGetTickFreq();

    if (freq == 0U) {
        return tick;
    }
    return (uint32_t)(((uint64_t)tick * 1000ULL) / (uint64_t)freq);
}

static uint8_t demo_elapsed_ms(uint32_t now_tick, uint32_t start_tick, uint32_t ms)
{
    return ((uint32_t)(now_tick - start_tick) >= demo_ms_to_ticks(ms)) ? 1U : 0U;
}

static uint8_t demo_cmd_type_to_sys_cmd(uint8_t type)
{
    switch ((demo_cmd_type_e)type) {
    case DEMO_CMD_STOP:
        return CTRL_SYS_CMD_DEMO_STOP;
    case DEMO_CMD_RUN:
        return CTRL_SYS_CMD_DEMO_RUN;
    case DEMO_CMD_HOME:
        return CTRL_SYS_CMD_DEMO_HOME;
    default:
        break;
    }

    return CTRL_SYS_CMD_RESERVED;
}

static void demo_submit_result(ctrl_result_e result,
                               ctrl_demo_reject_reason_e reject_reason,
                               demo_fault_e fault_reason,
                               uint32_t now_tick)
{
    ctrl_demo_result_payload_t payload = {0};

    payload.tick_ms = demo_tick_to_ms(now_tick);
    payload.request_seq = s_demo.request.seq;
    payload.request_cmd = demo_cmd_type_to_sys_cmd(s_demo.request.type);
    payload.request_source = s_demo.request.source;
    payload.result = (uint8_t)result;
    payload.reject_reason = (uint8_t)reject_reason;
    payload.fault_reason = (uint8_t)fault_reason;
    payload.state_after = (uint8_t)s_demo.state;
    payload.demo_id = s_demo.request.demo_id;
    payload.src_layer = s_demo.request.src_layer;
    payload.dst_layer = s_demo.request.dst_layer;
    payload.variant = s_demo.request.variant;
    payload.step_index = s_demo.step_index;
    payload.step_count = s_demo.step_count;

    (void)telemetry_submit_result(CTRL_TARGET_SYSTEM, CTRL_SYS_RPT_DEMO_RESULT,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
}

static void demo_submit_status(uint32_t now_tick)
{
    ctrl_demo_status_payload_t payload = {0};

    payload.tick_ms = demo_tick_to_ms(now_tick);
    payload.state = (uint8_t)s_demo.state;
    payload.fault = (uint8_t)s_demo.fault;
    payload.demo_id = s_demo.request.demo_id;
    payload.src_layer = s_demo.request.src_layer;
    payload.dst_layer = s_demo.request.dst_layer;
    payload.variant = s_demo.request.variant;
    payload.step_index = s_demo.step_index;
    payload.step_count = s_demo.step_count;
    payload.active = (s_demo.state == DEMO_STATE_RUNNING) ? 1U : 0U;

    (void)telemetry_submit_status(CTRL_TARGET_SYSTEM, CTRL_SYS_RPT_DEMO_STATUS,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
}

static uint8_t demo_get_pose(uint8_t pose_id, demo_pose_t *pose)
{
    if (pose == 0) {
        return 0U;
    }

    switch ((demo_pose_id_e)pose_id) {
    case DEMO_POSE_TOP_SAFE:
        *pose = (demo_pose_t){DEMO_TOP_SAFE_X_M, DEMO_TOP_SAFE_Y_M, DEMO_TOP_SAFE_Z_M, DEMO_TOP_SAFE_ROLL_RAD, DEMO_TOP_SAFE_PITCH_RAD};
        break;
    case DEMO_POSE_TOP_PICK_HOVER:
        *pose = (demo_pose_t){DEMO_TOP_PICK_HOVER_X_M, DEMO_TOP_PICK_HOVER_Y_M, DEMO_TOP_PICK_HOVER_Z_M, DEMO_TOP_PICK_HOVER_ROLL_RAD, DEMO_TOP_PICK_HOVER_PITCH_RAD};
        break;
    case DEMO_POSE_TOP_PICK_APPROACH:
        *pose = (demo_pose_t){DEMO_TOP_PICK_APPROACH_X_M, DEMO_TOP_PICK_APPROACH_Y_M, DEMO_TOP_PICK_APPROACH_Z_M, DEMO_TOP_PICK_APPROACH_ROLL_RAD, DEMO_TOP_PICK_APPROACH_PITCH_RAD};
        break;
    case DEMO_POSE_TOP_PICK_DOWN:
        *pose = (demo_pose_t){DEMO_TOP_PICK_DOWN_X_M, DEMO_TOP_PICK_DOWN_Y_M, DEMO_TOP_PICK_DOWN_Z_M, DEMO_TOP_PICK_DOWN_ROLL_RAD, DEMO_TOP_PICK_DOWN_PITCH_RAD};
        break;
    case DEMO_POSE_TOP_PICK_LIFT:
        *pose = (demo_pose_t){DEMO_TOP_PICK_LIFT_X_M, DEMO_TOP_PICK_LIFT_Y_M, DEMO_TOP_PICK_LIFT_Z_M, DEMO_TOP_PICK_LIFT_ROLL_RAD, DEMO_TOP_PICK_LIFT_PITCH_RAD};
        break;
    case DEMO_POSE_TOP_PLACE_APPROACH:
        *pose = (demo_pose_t){DEMO_TOP_PLACE_APPROACH_X_M, DEMO_TOP_PLACE_APPROACH_Y_M, DEMO_TOP_PLACE_APPROACH_Z_M, DEMO_TOP_PLACE_APPROACH_ROLL_RAD, DEMO_TOP_PLACE_APPROACH_PITCH_RAD};
        break;
    case DEMO_POSE_TOP_PLACE_DOWN:
        *pose = (demo_pose_t){DEMO_TOP_PLACE_DOWN_X_M, DEMO_TOP_PLACE_DOWN_Y_M, DEMO_TOP_PLACE_DOWN_Z_M, DEMO_TOP_PLACE_DOWN_ROLL_RAD, DEMO_TOP_PLACE_DOWN_PITCH_RAD};
        break;
    case DEMO_POSE_TOP_PLACE_LIFT:
        *pose = (demo_pose_t){DEMO_TOP_PLACE_LIFT_X_M, DEMO_TOP_PLACE_LIFT_Y_M, DEMO_TOP_PLACE_LIFT_Z_M, DEMO_TOP_PLACE_LIFT_ROLL_RAD, DEMO_TOP_PLACE_LIFT_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_SAFE:
        *pose = (demo_pose_t){DEMO_SIDE_SAFE_X_M, DEMO_SIDE_SAFE_Y_M, DEMO_SIDE_SAFE_Z_M, DEMO_SIDE_SAFE_ROLL_RAD, DEMO_SIDE_SAFE_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PICK_APPROACH:
        *pose = (demo_pose_t){DEMO_SIDE_PICK_APPROACH_X_M, DEMO_SIDE_PICK_APPROACH_Y_M, DEMO_SIDE_PICK_APPROACH_Z_M, DEMO_SIDE_PICK_APPROACH_ROLL_RAD, DEMO_SIDE_PICK_APPROACH_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PICK_CLEARANCE:
        *pose = (demo_pose_t){DEMO_SIDE_PICK_CLEARANCE_X_M, DEMO_SIDE_PICK_CLEARANCE_Y_M, DEMO_SIDE_PICK_CLEARANCE_Z_M, DEMO_SIDE_PICK_CLEARANCE_ROLL_RAD, DEMO_SIDE_PICK_CLEARANCE_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PICK_PRE_INSERT:
        *pose = (demo_pose_t){DEMO_SIDE_PICK_PRE_INSERT_X_M, DEMO_SIDE_PICK_PRE_INSERT_Y_M, DEMO_SIDE_PICK_PRE_INSERT_Z_M, DEMO_SIDE_PICK_PRE_INSERT_ROLL_RAD, DEMO_SIDE_PICK_PRE_INSERT_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PICK_INSERT:
        *pose = (demo_pose_t){DEMO_SIDE_PICK_INSERT_X_M, DEMO_SIDE_PICK_INSERT_Y_M, DEMO_SIDE_PICK_INSERT_Z_M, DEMO_SIDE_PICK_INSERT_ROLL_RAD, DEMO_SIDE_PICK_INSERT_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PICK_LIFT:
        *pose = (demo_pose_t){DEMO_SIDE_PICK_INSERT_X_M, DEMO_SIDE_PICK_INSERT_Y_M, DEMO_SIDE_PICK_LIFT_Z_M, DEMO_SIDE_PICK_INSERT_ROLL_RAD, DEMO_SIDE_PICK_INSERT_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_CARRY_SAFE:
        *pose = (demo_pose_t){DEMO_SIDE_CARRY_SAFE_X_M, DEMO_SIDE_CARRY_SAFE_Y_M, DEMO_SIDE_CARRY_SAFE_Z_M, DEMO_SIDE_CARRY_SAFE_ROLL_RAD, DEMO_SIDE_CARRY_SAFE_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PLACE_APPROACH:
        *pose = (demo_pose_t){DEMO_SIDE_PLACE_APPROACH_X_M, DEMO_SIDE_PLACE_APPROACH_Y_M, DEMO_SIDE_PLACE_APPROACH_Z_M, DEMO_SIDE_PLACE_APPROACH_ROLL_RAD, DEMO_SIDE_PLACE_APPROACH_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PLACE_CLEARANCE:
        *pose = (demo_pose_t){DEMO_SIDE_PLACE_CLEARANCE_X_M, DEMO_SIDE_PLACE_CLEARANCE_Y_M, DEMO_SIDE_PLACE_CLEARANCE_Z_M, DEMO_SIDE_PLACE_CLEARANCE_ROLL_RAD, DEMO_SIDE_PLACE_CLEARANCE_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PLACE_PRE_INSERT:
        *pose = (demo_pose_t){DEMO_SIDE_PLACE_PRE_INSERT_X_M, DEMO_SIDE_PLACE_PRE_INSERT_Y_M, DEMO_SIDE_PLACE_PRE_INSERT_Z_M, DEMO_SIDE_PLACE_PRE_INSERT_ROLL_RAD, DEMO_SIDE_PLACE_PRE_INSERT_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_PLACE_INSERT:
        *pose = (demo_pose_t){DEMO_SIDE_PLACE_INSERT_X_M, DEMO_SIDE_PLACE_INSERT_Y_M, DEMO_SIDE_PLACE_INSERT_Z_M, DEMO_SIDE_PLACE_INSERT_ROLL_RAD, DEMO_SIDE_PLACE_INSERT_PITCH_RAD};
        break;
    case DEMO_POSE_SIDE_RETREAT_SAFE:
        *pose = (demo_pose_t){DEMO_SIDE_RETREAT_SAFE_X_M, DEMO_SIDE_RETREAT_SAFE_Y_M, DEMO_SIDE_RETREAT_SAFE_Z_M, DEMO_SIDE_RETREAT_SAFE_ROLL_RAD, DEMO_SIDE_RETREAT_SAFE_PITCH_RAD};
        break;
    case DEMO_POSE_KEY_GRIP:
        *pose = (demo_pose_t){DEMO_KEY_GRIP_X_M, DEMO_KEY_GRIP_Y_M, DEMO_KEY_GRIP_Z_M, DEMO_KEY_GRIP_ROLL_RAD, DEMO_KEY_GRIP_PITCH_RAD};
        break;
    case DEMO_POSE_KEY_TURN:
        *pose = (demo_pose_t){DEMO_KEY_TURN_X_M, DEMO_KEY_TURN_Y_M, DEMO_KEY_TURN_Z_M, DEMO_KEY_TURN_ROLL_RAD, DEMO_KEY_TURN_PITCH_RAD};
        break;
    default:
        return 0U;
    }

    return 1U;
}

static uint8_t demo_variant_is_valid(uint8_t variant)
{
    switch ((ctrl_demo_variant_e)variant) {
    case CTRL_DEMO_VARIANT_AUTO:
    case CTRL_DEMO_VARIANT_DOWN:
    case CTRL_DEMO_VARIANT_UP:
        return 1U;
    default:
        break;
    }

    return 0U;
}

static uint8_t demo_resolve_lift_variant(const demo_cmd_msg_t *msg)
{
    if (msg == 0) {
        return CTRL_DEMO_VARIANT_DOWN;
    }
    if (msg->variant != CTRL_DEMO_VARIANT_AUTO) {
        return msg->variant;
    }
    if (msg->demo_id == DEMO_ID_LAYER_TRANSFER) {
        if (msg->dst_layer > msg->src_layer) {
            return CTRL_DEMO_VARIANT_UP;
        }
        if (msg->dst_layer < msg->src_layer) {
            return CTRL_DEMO_VARIANT_DOWN;
        }
    }

    return CTRL_DEMO_VARIANT_DOWN;
}

static uint8_t demo_layer_to_z_for_variant(uint8_t layer, uint8_t variant, float *z)
{
    if (z == 0) {
        return 0U;
    }

    if (variant == CTRL_DEMO_VARIANT_UP) {
        switch (layer) {
        case 1U:
            *z = DEMO_LAYER_1_UP_Z_M;
            return 1U;
        case 2U:
            *z = DEMO_LAYER_2_UP_Z_M;
            return 1U;
        case 3U:
            *z = DEMO_LAYER_3_UP_Z_M;
            return 1U;
        default:
            break;
        }
    }

    switch (layer) {
    case 1U:
        *z = DEMO_LAYER_1_DOWN_Z_M;
        return 1U;
    case 2U:
        *z = DEMO_LAYER_2_DOWN_Z_M;
        return 1U;
    case 3U:
        *z = DEMO_LAYER_3_DOWN_Z_M;
        return 1U;
    default:
        break;
    }

    return 0U;
}

static uint8_t demo_put_chassis_stop(uint32_t now_tick)
{
    chassis_cmd_msg_t msg = {0};

    msg.type = CHASSIS_CMD_STOP;
    msg.source = CTRL_SRC_MCU;
    msg.tick = now_tick;

    return (osMessageQueuePut(ChassisCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static const float s_demo_chassis_dirs[5] = {0.0f, 1.570796f, 3.141593f, -1.570796f, -2.356194f};

static uint8_t demo_put_chassis_mov(uint8_t dir_idx, float speed, uint32_t now_tick)
{
    chassis_cmd_msg_t msg = {0};
    float dir;

    if (dir_idx >= 5U) {
        return 0U;
    }
    dir = s_demo_chassis_dirs[dir_idx];

    msg.type = CHASSIS_CMD_MOV;
    msg.source = CTRL_SRC_MCU;
    msg.move_cs = CTRL_CHS_MOVE_LCS;
    msg.direction = dir;
    msg.v = speed;
    msg.omega = 0.0f;
    msg.tick = now_tick;

    return (osMessageQueuePut(ChassisCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_chassis_mov_arc(uint8_t dir_idx, float speed, float omega, uint32_t now_tick)
{
    chassis_cmd_msg_t msg = {0};
    float dir;

    if (dir_idx >= 5U) {
        return 0U;
    }
    dir = s_demo_chassis_dirs[dir_idx];

    msg.type = CHASSIS_CMD_MOV;
    msg.source = CTRL_SRC_MCU;
    msg.move_cs = CTRL_CHS_MOVE_LCS;
    msg.direction = dir;
    msg.v = speed;
    msg.omega = omega;
    msg.tick = now_tick;

    return (osMessageQueuePut(ChassisCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_arm_home(uint32_t now_tick)
{
    arm_cmd_msg_t msg = {0};

    msg.type = ARM_CMD_HOME;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.tick = now_tick;

    return (osMessageQueuePut(ArmCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_arm_stop(uint32_t now_tick)
{
    arm_cmd_msg_t msg = {0};

    msg.type = ARM_CMD_STOP;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.tick = now_tick;

    return (osMessageQueuePut(ArmCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_arm_pose(uint8_t pose_id, uint32_t now_tick)
{
    demo_pose_t pose;
    arm_cmd_msg_t msg = {0};

    if (demo_get_pose(pose_id, &pose) == 0U) {
        return 0U;
    }

    msg.type = ARM_CMD_MOVE_POSE;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.x = pose.x;
    msg.y = pose.y;
    msg.z = pose.z;
    msg.roll = pose.roll;
    msg.pitch = pose.pitch;
    msg.tick = now_tick;

    return (osMessageQueuePut(ArmCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_arm_gripper(float gripper_rad, uint32_t now_tick)
{
    arm_cmd_msg_t msg = {0};

    msg.type = ARM_CMD_GRIPPER;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.gripper_rad = gripper_rad;
    msg.tick = now_tick;

    return (osMessageQueuePut(ArmCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_lift_home(uint32_t now_tick)
{
    lift_cmd_msg_t msg = {0};

    msg.type = LIFT_CMD_HOME;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.tick = now_tick;

    return (osMessageQueuePut(LiftCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_lift_stop(uint32_t now_tick)
{
    lift_cmd_msg_t msg = {0};

    msg.type = LIFT_CMD_STOP;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.tick = now_tick;

    return (osMessageQueuePut(LiftCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_lift_layer(uint8_t layer_select, uint32_t now_tick)
{
    lift_cmd_msg_t msg = {0};
    uint8_t layer = (layer_select == DEMO_LAYER_SELECT_DST) ? s_demo.request.dst_layer : s_demo.request.src_layer;
    uint8_t variant = demo_resolve_lift_variant(&s_demo.request);

    if (demo_layer_to_z_for_variant(layer, variant, &msg.z) == 0U) {
        s_demo.fault = DEMO_FAULT_BAD_LAYER;
        return 0U;
    }

    msg.type = LIFT_CMD_MOVE_Z;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.tick = now_tick;

    return (osMessageQueuePut(LiftCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static uint8_t demo_put_lift_z(float z, uint32_t now_tick)
{
    lift_cmd_msg_t msg = {0};

    msg.type = LIFT_CMD_MOVE_Z;
    msg.source = CTRL_SRC_MCU;
    msg.seq = ++s_demo.internal_seq;
    msg.z = z;
    msg.tick = now_tick;

    return (osMessageQueuePut(LiftCmdQueueHandle, &msg, 0U, DEMO_CMD_QUEUE_WAIT) == osOK) ? 1U : 0U;
}

static void demo_stop_all(uint32_t now_tick)
{
    (void)demo_put_chassis_stop(now_tick);
    (void)demo_put_arm_stop(now_tick);
    (void)demo_put_lift_stop(now_tick);
}

static uint8_t demo_select_steps(uint8_t demo_id,
                                 const demo_step_t **steps,
                                 uint8_t *step_count)
{
    if ((steps == 0) || (step_count == 0)) {
        return 0U;
    }

    switch ((demo_id_e)demo_id) {
    case DEMO_ID_STATIC_PICK_PLACE:
        *steps = s_demo_static_top_steps;
        *step_count = (uint8_t)(sizeof(s_demo_static_top_steps) / sizeof(s_demo_static_top_steps[0]));
        return 1U;
    case DEMO_ID_LAYER_PICK:
        *steps = s_demo_layer_pick_steps;
        *step_count = (uint8_t)(sizeof(s_demo_layer_pick_steps) / sizeof(s_demo_layer_pick_steps[0]));
        return 1U;
    case DEMO_ID_LAYER_PLACE:
        *steps = s_demo_layer_place_steps;
        *step_count = (uint8_t)(sizeof(s_demo_layer_place_steps) / sizeof(s_demo_layer_place_steps[0]));
        return 1U;
    case DEMO_ID_LAYER_TRANSFER:
        *steps = s_demo_layer_transfer_steps;
        *step_count = (uint8_t)(sizeof(s_demo_layer_transfer_steps) / sizeof(s_demo_layer_transfer_steps[0]));
        return 1U;
    case DEMO_ID_KEY_TURN:
        *steps = s_demo_key_turn_steps;
        *step_count = (uint8_t)(sizeof(s_demo_key_turn_steps) / sizeof(s_demo_key_turn_steps[0]));
        return 1U;
    case DEMO_ID_CABINET_PULL:
        *steps = s_demo_cabinet_pull_steps;
        *step_count = (uint8_t)(sizeof(s_demo_cabinet_pull_steps) / sizeof(s_demo_cabinet_pull_steps[0]));
        return 1U;
    default:
        break;
    }

    return 0U;
}

static uint8_t demo_request_layers_are_valid(const demo_cmd_msg_t *msg)
{
    float z;

    if (msg == 0) {
        return 0U;
    }

    if ((msg->demo_id == DEMO_ID_STATIC_PICK_PLACE) ||
        (msg->demo_id == DEMO_ID_KEY_TURN) ||
        (msg->demo_id == DEMO_ID_CABINET_PULL)) {
        return 1U;
    }
    if (demo_variant_is_valid(msg->variant) == 0U) {
        return 0U;
    }
    if ((msg->demo_id == DEMO_ID_LAYER_PICK) ||
        (msg->demo_id == DEMO_ID_LAYER_TRANSFER)) {
        if (demo_layer_to_z_for_variant(msg->src_layer, demo_resolve_lift_variant(msg), &z) == 0U) {
            return 0U;
        }
    }
    if ((msg->demo_id == DEMO_ID_LAYER_PLACE) ||
        (msg->demo_id == DEMO_ID_LAYER_TRANSFER)) {
        if (demo_layer_to_z_for_variant(msg->dst_layer, demo_resolve_lift_variant(msg), &z) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static void demo_start(const demo_cmd_msg_t *msg, uint32_t now_tick)
{
    const demo_step_t *steps = 0;
    uint8_t step_count = 0U;

    if (msg == 0) {
        return;
    }
    s_demo.request = *msg;
    s_demo.terminal_result_sent = 0U;

    if (demo_select_steps(msg->demo_id, &steps, &step_count) == 0U) {
        s_demo.state = DEMO_STATE_FAULT;
        s_demo.fault = DEMO_FAULT_UNKNOWN_DEMO;
        s_demo.state_start_tick = now_tick;
        demo_submit_result(CTRL_RESULT_FAILED, CTRL_DEMO_REJECT_NONE,
                           s_demo.fault, now_tick);
        s_demo.terminal_result_sent = 1U;
        return;
    }
    if (demo_request_layers_are_valid(msg) == 0U) {
        s_demo.state = DEMO_STATE_FAULT;
        s_demo.fault = (demo_variant_is_valid(msg->variant) == 0U) ? DEMO_FAULT_BAD_VARIANT : DEMO_FAULT_BAD_LAYER;
        s_demo.state_start_tick = now_tick;
        demo_submit_result(CTRL_RESULT_FAILED, CTRL_DEMO_REJECT_NONE,
                           s_demo.fault, now_tick);
        s_demo.terminal_result_sent = 1U;
        return;
    }

    s_demo.steps = steps;
    s_demo.step_count = step_count;
    s_demo.step_index = 0U;
    s_demo.step_sent = 0U;
    s_demo.step_start_tick = now_tick;
    s_demo.state_start_tick = now_tick;
    s_demo.state = DEMO_STATE_RUNNING;
    s_demo.fault = DEMO_FAULT_NONE;
    s_demo.terminal_result_sent = 0U;
}

static void demo_start_home(const demo_cmd_msg_t *request, uint32_t now_tick)
{
    demo_cmd_msg_t msg = {0};

    msg.type = DEMO_CMD_HOME;
    if (request != 0) {
        msg.source = request->source;
        msg.seq = request->seq;
    } else {
        msg.source = CTRL_SRC_MCU;
    }
    msg.demo_id = DEMO_ID_NONE;
    msg.tick = now_tick;

    s_demo.request = msg;
    s_demo.steps = s_demo_home_steps;
    s_demo.step_count = (uint8_t)(sizeof(s_demo_home_steps) / sizeof(s_demo_home_steps[0]));
    s_demo.step_index = 0U;
    s_demo.step_sent = 0U;
    s_demo.step_start_tick = now_tick;
    s_demo.state_start_tick = now_tick;
    s_demo.state = DEMO_STATE_RUNNING;
    s_demo.fault = DEMO_FAULT_NONE;
    s_demo.terminal_result_sent = 0U;
}

static void demo_enter_fault(demo_fault_e fault, uint32_t now_tick)
{
    s_demo.state = DEMO_STATE_FAULT;
    s_demo.fault = fault;
    s_demo.state_start_tick = now_tick;
    demo_stop_all(now_tick);
    if (s_demo.terminal_result_sent == 0U) {
        demo_submit_result(CTRL_RESULT_FAILED, CTRL_DEMO_REJECT_NONE,
                           fault, now_tick);
        s_demo.terminal_result_sent = 1U;
    }
}

static void demo_abort(uint32_t now_tick)
{
    s_demo.state = DEMO_STATE_ABORT;
    s_demo.fault = DEMO_FAULT_NONE;
    s_demo.state_start_tick = now_tick;
    demo_stop_all(now_tick);
    if (s_demo.terminal_result_sent == 0U) {
        demo_submit_result(CTRL_RESULT_ABORTED, CTRL_DEMO_REJECT_NONE,
                           DEMO_FAULT_NONE, now_tick);
        s_demo.terminal_result_sent = 1U;
    }
}

static void demo_advance_step(uint32_t now_tick)
{
    s_demo.step_index++;
    s_demo.step_sent = 0U;
    s_demo.step_start_tick = now_tick;
}

static uint8_t demo_step_send(const demo_step_t *step, uint32_t now_tick)
{
    if (step == 0) {
        return 0U;
    }

    switch (step->type) {
    case DEMO_STEP_STOP_CHASSIS:
        return demo_put_chassis_stop(now_tick);
    case DEMO_STEP_CHASSIS_MOV:
        return demo_put_chassis_mov(step->arg, step->value, now_tick);
    case DEMO_STEP_CHASSIS_MOV_ARC:
        return demo_put_chassis_mov_arc(step->arg, step->value, DEMO_KEY_PULL_OMEGA_RADPS, now_tick);
    case DEMO_STEP_ARM_HOME:
        return demo_put_arm_home(now_tick);
    case DEMO_STEP_ARM_POSE:
        return demo_put_arm_pose(step->arg, now_tick);
    case DEMO_STEP_ARM_GRIPPER:
        return demo_put_arm_gripper(step->value, now_tick);
    case DEMO_STEP_LIFT_HOME_IF_NEEDED:
        if (g_lift_state.runtime.is_homed != 0U) {
            return 2U;
        }
        return demo_put_lift_home(now_tick);
    case DEMO_STEP_LIFT_LAYER:
        return demo_put_lift_layer(step->arg, now_tick);
    case DEMO_STEP_LIFT_Z:
        return demo_put_lift_z(step->value, now_tick);
    case DEMO_STEP_DELAY:
        return 1U;
    default:
        break;
    }

    return 0U;
}

static uint8_t demo_arm_step_complete(const demo_step_t *step, uint32_t now_tick)
{
    if (g_arm_state.runtime.has_fault != 0U) {
        demo_enter_fault(DEMO_FAULT_ARM, now_tick);
        return 0U;
    }
    if (demo_elapsed_ms(now_tick, s_demo.step_start_tick, step->timeout_ms) != 0U) {
        demo_enter_fault(DEMO_FAULT_TIMEOUT, now_tick);
        return 0U;
    }
    if (demo_elapsed_ms(now_tick, s_demo.step_start_tick, DEMO_STEP_MIN_WAIT_MS) == 0U) {
        return 0U;
    }

    return ((g_arm_state.runtime.is_busy == 0U) &&
            (g_arm_state.runtime.is_reached != 0U))
               ? 1U
               : 0U;
}

static uint8_t demo_lift_step_complete(const demo_step_t *step, uint32_t now_tick)
{
    if (g_lift_state.runtime.has_fault != 0U) {
        demo_enter_fault(DEMO_FAULT_LIFT, now_tick);
        return 0U;
    }
    if (demo_elapsed_ms(now_tick, s_demo.step_start_tick, step->timeout_ms) != 0U) {
        demo_enter_fault(DEMO_FAULT_TIMEOUT, now_tick);
        return 0U;
    }
    if (demo_elapsed_ms(now_tick, s_demo.step_start_tick, DEMO_STEP_MIN_WAIT_MS) == 0U) {
        return 0U;
    }

    return (g_lift_state.runtime.is_busy == 0U) ? 1U : 0U;
}

static uint8_t demo_step_complete(const demo_step_t *step, uint32_t now_tick)
{
    if (step == 0) {
        return 0U;
    }

    switch (step->type) {
    case DEMO_STEP_STOP_CHASSIS:
        return demo_elapsed_ms(now_tick, s_demo.step_start_tick,
                               step->timeout_ms);
    case DEMO_STEP_CHASSIS_MOV:
        if (demo_elapsed_ms(now_tick, s_demo.step_start_tick,
                            step->timeout_ms)) {
            (void)demo_put_chassis_stop(now_tick);
            return 1U;
        }
        /* Keep refreshing MOV to prevent chassis control timeout (300ms). */
        (void)demo_put_chassis_mov(step->arg, step->value, now_tick);
        return 0U;
    case DEMO_STEP_CHASSIS_MOV_ARC:
        if (demo_elapsed_ms(now_tick, s_demo.step_start_tick,
                            step->timeout_ms)) {
            (void)demo_put_chassis_stop(now_tick);
            return 1U;
        }
        (void)demo_put_chassis_mov_arc(step->arg, step->value, DEMO_KEY_PULL_OMEGA_RADPS, now_tick);
        return 0U;
    case DEMO_STEP_ARM_HOME:
    case DEMO_STEP_ARM_POSE:
    case DEMO_STEP_ARM_GRIPPER:
        return demo_arm_step_complete(step, now_tick);
    case DEMO_STEP_LIFT_HOME_IF_NEEDED:
    case DEMO_STEP_LIFT_LAYER:
    case DEMO_STEP_LIFT_Z:
        return demo_lift_step_complete(step, now_tick);
    case DEMO_STEP_DELAY:
        return demo_elapsed_ms(now_tick, s_demo.step_start_tick,
                               step->timeout_ms);
    default:
        break;
    }

    return 0U;
}

static void demo_update_running(uint32_t now_tick)
{
    const demo_step_t *step;
    uint8_t send_status;

    if ((s_demo.steps == 0) || (s_demo.step_index >= s_demo.step_count)) {
        s_demo.state = DEMO_STATE_DONE;
        s_demo.state_start_tick = now_tick;
        if (s_demo.terminal_result_sent == 0U) {
            demo_submit_result(CTRL_RESULT_COMPLETED, CTRL_DEMO_REJECT_NONE,
                               DEMO_FAULT_NONE, now_tick);
            s_demo.terminal_result_sent = 1U;
        }
        return;
    }

    step = &s_demo.steps[s_demo.step_index];
    if (s_demo.step_sent == 0U) {
        send_status = demo_step_send(step, now_tick);
        if (send_status == 0U) {
            demo_enter_fault((s_demo.fault != DEMO_FAULT_NONE) ? s_demo.fault : DEMO_FAULT_QUEUE_FULL,
                             now_tick);
            return;
        }
        if (send_status == 2U) {
            demo_advance_step(now_tick);
            return;
        }
        s_demo.step_sent = 1U;
        s_demo.step_start_tick = now_tick;
        return;
    }

    if (demo_step_complete(step, now_tick) != 0U) {
        demo_advance_step(now_tick);
    }
}

static void demo_handle_msg(const demo_cmd_msg_t *msg, uint32_t now_tick)
{
    if (msg == 0) {
        return;
    }

    switch ((demo_cmd_type_e)msg->type) {
    case DEMO_CMD_STOP:
        s_demo.request = *msg;
        demo_abort(now_tick);
        break;
    case DEMO_CMD_RUN:
        if ((s_demo.state == DEMO_STATE_IDLE) ||
            (s_demo.state == DEMO_STATE_DONE) ||
            (s_demo.state == DEMO_STATE_ABORT) ||
            (s_demo.state == DEMO_STATE_FAULT)) {
            demo_start(msg, now_tick);
        } else {
            demo_cmd_msg_t busy_msg = *msg;
            demo_cmd_msg_t active_msg = s_demo.request;
            s_demo.request = busy_msg;
            demo_submit_result(CTRL_RESULT_REJECTED, CTRL_DEMO_REJECT_BUSY,
                               DEMO_FAULT_NONE, now_tick);
            s_demo.request = active_msg;
        }
        break;
    case DEMO_CMD_HOME:
        if ((s_demo.state == DEMO_STATE_IDLE) ||
            (s_demo.state == DEMO_STATE_DONE) ||
            (s_demo.state == DEMO_STATE_ABORT) ||
            (s_demo.state == DEMO_STATE_FAULT)) {
            demo_start_home(msg, now_tick);
        } else {
            demo_cmd_msg_t busy_msg = *msg;
            demo_cmd_msg_t active_msg = s_demo.request;
            s_demo.request = busy_msg;
            demo_submit_result(CTRL_RESULT_REJECTED, CTRL_DEMO_REJECT_BUSY,
                               DEMO_FAULT_NONE, now_tick);
            s_demo.request = active_msg;
        }
        break;
    default:
        break;
    }
}

static void demo_drain_queue(uint32_t now_tick)
{
    demo_cmd_msg_t msg;

    while (osMessageQueueGet(DemoCmdQueueHandle, &msg, 0U, 0U) == osOK) {
        demo_handle_msg(&msg, now_tick);
    }
}

static void demo_update_snapshot(uint32_t now_tick)
{
    g_demo_state.tick_ms = demo_tick_to_ms(now_tick);
    g_demo_state.state = s_demo.state;
    g_demo_state.fault = s_demo.fault;
    g_demo_state.demo_id = s_demo.request.demo_id;
    g_demo_state.step_index = s_demo.step_index;
    g_demo_state.step_count = s_demo.step_count;
    g_demo_state.active = (s_demo.state == DEMO_STATE_RUNNING) ? 1U : 0U;

    if (demo_elapsed_ms(now_tick, s_demo.status_last_tick, 50U) != 0U) {
        s_demo.status_last_tick = now_tick;
        demo_submit_status(now_tick);
    }
}

void StartDemoTask(void *argument)
{
    uint32_t now_tick;

    (void)argument;

    s_demo.state = DEMO_STATE_IDLE;
    s_demo.fault = DEMO_FAULT_NONE;
    s_demo.status_last_tick = osKernelGetTickCount();

    for (;;) {
        now_tick = osKernelGetTickCount();

        demo_drain_queue(now_tick);

        switch (s_demo.state) {
        case DEMO_STATE_RUNNING:
            demo_update_running(now_tick);
            break;
        case DEMO_STATE_DONE:
        case DEMO_STATE_ABORT:
        case DEMO_STATE_FAULT:
            if (demo_elapsed_ms(now_tick, s_demo.state_start_tick, 500U) != 0U) {
                s_demo.state = DEMO_STATE_IDLE;
                s_demo.step_index = 0U;
                s_demo.step_count = 0U;
                s_demo.step_sent = 0U;
            }
            break;
        case DEMO_STATE_IDLE:
        default:
            break;
        }

        demo_update_snapshot(now_tick);
        osDelayUntil(now_tick + demo_ms_to_ticks(DEMO_TASK_PERIOD_MS));
    }
}
