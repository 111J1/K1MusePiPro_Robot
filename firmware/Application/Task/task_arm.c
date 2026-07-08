#include "task_arm.h"

#include "cmsis_os.h"
#include "dev_sts_servo.h"
#include "drv_uart.h"
#include "mdl_control_arbitration.h"
#include "mdl_so101_arm.h"
#include "mdl_so101_arm_config.h"
#include "task_telemetry.h"
#include <string.h>

#define ARM_TASK_PERIOD_MS (10U)
#define ARM_STATUS_PERIOD_MS (50UL)
#define ARM_STATUS_PHASE_MS (20UL)
#define ARM_TASK_DT_S (0.01f)
#define ARM_TASK_BOOT_WAIT_MS (500UL)
#define ARM_TASK_HOME_TIMEOUT_MS (10000UL)
#define ARM_CONTROL_TIMEOUT_MS (10000UL)
#define ARM_TASK_FATAL_TIMEOUT_COUNT (20UL)
#define ARM_TASK_OVER_TEMP_CONFIRM_MS (5000UL)
#define ARM_TASK_VOLTAGE_CONFIRM_MS (2000UL)
#define ARM_TASK_BUS_TIMEOUT_CONFIRM_MS (2000UL)
#define ARM_TASK_STATUS_ERROR_CONFIRM_MS (1000UL)
#define ARM_TASK_GRIPPER_REACHED_EPS_RAD (0.20f)

#define ARM_TASK_VOLTAGE_FAULT_MASK  \
    (STS_SERVO_FAULT_UNDER_VOLTAGE | \
     STS_SERVO_FAULT_OVER_VOLTAGE)

typedef struct {
    uint8_t valid;
    uint8_t seq;
    uint8_t cmd;
    uint8_t source;
    float requested_x;
    float requested_y;
    float requested_z;
    float requested_gripper_rad;
    float accepted_x;
    float accepted_y;
    float accepted_z;
    float accepted_gripper_rad;
} arm_active_cmd_t;

static void arm_servo_group_update(void *ctx);
static void arm_task_change_state(arm_task_state_t state, uint32_t now);
static void arm_task_state_snapshot_update(uint32_t now);
static void arm_task_send_status(void);
static void arm_task_hold_current_pose(uint32_t now);

static sts_servo_bus_t s_servo_bus;

static sts_servo_t s_servo[SO101_ARM_SERVO_COUNT] = {
    [0] = {
        .id = SO101_ARM_SERVO_1_ID,
        .position_min = SO101_ARM_SERVO_1_POSITION_MIN,
        .position_max = SO101_ARM_SERVO_1_POSITION_MAX,
        .position_zero = SO101_ARM_SERVO_1_POSITION_ZERO,
        .position_per_rad = SO101_ARM_SERVO_POSITION_PER_RAD,
        .direction = SO101_ARM_SERVO_1_DIRECTION,
        .angle_min_rad = SO101_ARM_JOINT_1_MIN_RAD,
        .angle_max_rad = SO101_ARM_JOINT_1_MAX_RAD,
        .default_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .command_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .default_acc = SO101_ARM_SERVO_DEFAULT_ACC,
        .timeout_ms = SO101_ARM_SERVO_TIMEOUT_MS,
        .command_time = 0U,
    },
    [1] = {
        .id = SO101_ARM_SERVO_2_ID,
        .position_min = SO101_ARM_SERVO_2_POSITION_MIN,
        .position_max = SO101_ARM_SERVO_2_POSITION_MAX,
        .position_zero = SO101_ARM_SERVO_2_POSITION_ZERO,
        .position_per_rad = SO101_ARM_SERVO_POSITION_PER_RAD,
        .direction = SO101_ARM_SERVO_2_DIRECTION,
        .angle_min_rad = SO101_ARM_JOINT_2_MIN_RAD,
        .angle_max_rad = SO101_ARM_JOINT_2_MAX_RAD,
        .default_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .command_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .default_acc = SO101_ARM_SERVO_DEFAULT_ACC,
        .timeout_ms = SO101_ARM_SERVO_TIMEOUT_MS,
        .command_time = 0U,
    },
    [2] = {
        .id = SO101_ARM_SERVO_3_ID,
        .position_min = SO101_ARM_SERVO_3_POSITION_MIN,
        .position_max = SO101_ARM_SERVO_3_POSITION_MAX,
        .position_zero = SO101_ARM_SERVO_3_POSITION_ZERO,
        .position_per_rad = SO101_ARM_SERVO_POSITION_PER_RAD,
        .direction = SO101_ARM_SERVO_3_DIRECTION,
        .angle_min_rad = SO101_ARM_JOINT_3_MIN_RAD,
        .angle_max_rad = SO101_ARM_JOINT_3_MAX_RAD,
        .default_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .command_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .default_acc = SO101_ARM_SERVO_DEFAULT_ACC,
        .timeout_ms = SO101_ARM_SERVO_TIMEOUT_MS,
        .command_time = 0U,
    },
    [3] = {
        .id = SO101_ARM_SERVO_4_ID,
        .position_min = SO101_ARM_SERVO_4_POSITION_MIN,
        .position_max = SO101_ARM_SERVO_4_POSITION_MAX,
        .position_zero = SO101_ARM_SERVO_4_POSITION_ZERO,
        .position_per_rad = SO101_ARM_SERVO_POSITION_PER_RAD,
        .direction = SO101_ARM_SERVO_4_DIRECTION,
        .angle_min_rad = SO101_ARM_JOINT_4_MIN_RAD,
        .angle_max_rad = SO101_ARM_JOINT_4_MAX_RAD,
        .default_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .command_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .default_acc = SO101_ARM_SERVO_DEFAULT_ACC,
        .timeout_ms = SO101_ARM_SERVO_TIMEOUT_MS,
        .command_time = 0U,
    },
    [4] = {
        .id = SO101_ARM_SERVO_5_ID,
        .position_min = SO101_ARM_SERVO_5_POSITION_MIN,
        .position_max = SO101_ARM_SERVO_5_POSITION_MAX,
        .position_zero = SO101_ARM_SERVO_5_POSITION_ZERO,
        .position_per_rad = SO101_ARM_SERVO_POSITION_PER_RAD,
        .direction = SO101_ARM_SERVO_5_DIRECTION,
        .angle_min_rad = SO101_ARM_JOINT_5_MIN_RAD,
        .angle_max_rad = SO101_ARM_JOINT_5_MAX_RAD,
        .default_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .command_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .default_acc = SO101_ARM_SERVO_DEFAULT_ACC,
        .timeout_ms = SO101_ARM_SERVO_TIMEOUT_MS,
        .command_time = 0U,
    },
    [5] = {
        .id = SO101_ARM_GRIPPER_SERVO_ID,
        .position_min = SO101_ARM_GRIPPER_POSITION_MIN,
        .position_max = SO101_ARM_GRIPPER_POSITION_MAX,
        .position_zero = STS_SERVO_POSITION_ZERO_DEFAULT,
        .position_per_rad = STS_SERVO_POSITION_PER_RAD_DEFAULT,
        .direction = SO101_ARM_GRIPPER_DIRECTION,
        .angle_min_rad = SO101_ARM_GRIPPER_MIN_RAD,
        .angle_max_rad = SO101_ARM_GRIPPER_MAX_RAD,
        .default_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .command_speed = SO101_ARM_SERVO_DEFAULT_SPEED,
        .default_acc = SO101_ARM_SERVO_DEFAULT_ACC,
        .timeout_ms = SO101_ARM_SERVO_TIMEOUT_MS,
        .command_time = 0U,
    },
};

static so101_arm_servo_group_t s_servo_group = {
    .bus = &s_servo_bus,
    .servos = s_servo,
    .servo_count = SO101_ARM_SERVO_COUNT,
    .uart = &uart1_driver,
    .timeout_ms = SO101_ARM_BUS_TIMEOUT_MS,
};

static so101_arm_t s_arm = {
    .joint[0] = {
        .ctx = &s_servo[0],
        .init = 0,
        .set_angle_rad = sts_servo_set_angle_rad,
        .get_angle_rad = sts_servo_get_angle_rad,
        .enable_torque = sts_servo_enable_torque,
        .update = 0,
        .get_faults = sts_servo_get_faults,
    },
    .joint[1] = {
        .ctx = &s_servo[1],
        .init = 0,
        .set_angle_rad = sts_servo_set_angle_rad,
        .get_angle_rad = sts_servo_get_angle_rad,
        .enable_torque = sts_servo_enable_torque,
        .update = 0,
        .get_faults = sts_servo_get_faults,
    },
    .joint[2] = {
        .ctx = &s_servo[2],
        .init = 0,
        .set_angle_rad = sts_servo_set_angle_rad,
        .get_angle_rad = sts_servo_get_angle_rad,
        .enable_torque = sts_servo_enable_torque,
        .update = 0,
        .get_faults = sts_servo_get_faults,
    },
    .joint[3] = {
        .ctx = &s_servo[3],
        .init = 0,
        .set_angle_rad = sts_servo_set_angle_rad,
        .get_angle_rad = sts_servo_get_angle_rad,
        .enable_torque = sts_servo_enable_torque,
        .update = 0,
        .get_faults = sts_servo_get_faults,
    },
    .joint[4] = {
        .ctx = &s_servo[4],
        .init = 0,
        .set_angle_rad = sts_servo_set_angle_rad,
        .get_angle_rad = sts_servo_get_angle_rad,
        .enable_torque = sts_servo_enable_torque,
        .update = 0,
        .get_faults = sts_servo_get_faults,
    },
    .max_joint_step_rad = SO101_ARM_MAX_JOINT_STEP_RAD,
    .reached_angle_eps_rad = SO101_ARM_REACHED_ANGLE_EPS_RAD,
    .reached_pos_eps_m = SO101_ARM_REACHED_POS_EPS_M,
    .nonfatal_joint_fault_mask = STS_SERVO_FAULT_POSITION_ERROR | STS_SERVO_FAULT_OVER_LOAD | STS_SERVO_FAULT_OVER_CURRENT,
    .joint_group = {
        .ctx = &s_servo_group,
        .init = so101_arm_servo_group_init,
        .update = arm_servo_group_update,
        .enable_torque = so101_arm_servo_group_enable_torque,
    },
    .gripper = {
        .ctx = &s_servo[SO101_ARM_GRIPPER_INDEX],
        .init = 0,
        .set_angle_rad = sts_servo_set_angle_rad,
        .get_angle_rad = sts_servo_get_angle_rad,
        .enable_torque = sts_servo_enable_torque,
        .get_faults = sts_servo_get_faults,
    },
};

static uint32_t s_state_start_ms = 0UL;
static uint32_t s_consecutive_timeout_count = 0UL;
static uint32_t s_over_temp_start_ms = 0UL;
static uint32_t s_voltage_fault_start_ms = 0UL;
static uint32_t s_bus_timeout_start_ms = 0UL;
static uint32_t s_status_error_start_ms = 0UL;
static uint8_t s_home_started = 0U;
static so101_arm_status_t s_last_mdl_status = SO101_ARM_OK;
static arm_fault_source_t s_arm_fault_source = ARM_FAULT_SRC_NONE;
static module_control_arbitration_t arm_arbitration;
static uint32_t s_arm_status_phase_start_ms = 0UL;
static uint32_t s_arm_status_last_ms = 0UL;
static uint8_t s_arm_status_phase_ready = 0U;
static arm_active_cmd_t s_arm_active_cmd;
static float s_arm_target_gripper_rad = 0.0f;
static uint8_t s_home_feedback_wait_started = 0U;
static uint8_t s_home_hold_target_started = 0U;

volatile arm_task_state_t g_arm_task_state = ARM_TASK_INIT;
volatile arm_task_status_t g_arm_task_status = ARM_TASK_BUSY;
volatile arm_state_snapshot_t g_arm_state;

static uint8_t arm_task_can_accept_motion_command(void)
{
    return ((g_arm_task_state == ARM_TASK_IDLE) ||
            (g_arm_task_state == ARM_TASK_EXECUTE) ||
            (g_arm_task_state == ARM_TASK_WAIT_TARGET) ||
            (g_arm_task_state == ARM_TASK_REACHED))
               ? 1U
               : 0U;
}

arm_task_state_t arm_task_get_state(void)
{
    return g_arm_task_state;
}

arm_task_status_t arm_task_get_status(void)
{
    return g_arm_task_status;
}

uint8_t arm_task_is_ready(void)
{
    return arm_task_can_accept_motion_command();
}

uint32_t sts_servo_platform_get_tick_ms(void)
{
    uint32_t tick_freq = osKernelGetTickFreq();

    if (tick_freq == 0U) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)osKernelGetTickCount() * 1000ULL) / tick_freq);
}

static uint32_t arm_elapsed_ms(uint32_t now, uint32_t start)
{
    return (uint32_t)(now - start);
}

static void arm_task_change_state(arm_task_state_t state, uint32_t now)
{
    g_arm_task_state = state;
    s_state_start_ms = now;
}

static void arm_servo_group_update(void *ctx)
{
    so101_arm_servo_group_t *group = (so101_arm_servo_group_t *)ctx;
    sts_servo_status_t status;

    if ((group == 0) || (group->bus == 0) || (group->servos == 0)) {
        return;
    }

    status = sts_servo_update_sync(group->bus, group->servos, group->servo_count);

    if (status == STS_SERVO_ERR_TIMEOUT) {
        s_consecutive_timeout_count++;
    } else if (status == STS_SERVO_OK) {
        s_consecutive_timeout_count = 0UL;
    }
}

static uint32_t arm_task_collect_faults(void)
{
    uint32_t faults = STS_SERVO_FAULT_NONE;

    for (uint8_t i = 0U; i < SO101_ARM_SERVO_COUNT; i++) {
        faults |= s_servo[i].fault_flags;
    }

    return faults;
}

static void arm_task_enter_fault(arm_task_status_t status)
{
    if (s_arm_fault_source == ARM_FAULT_SRC_NONE) {
        s_arm_fault_source = (status == ARM_TASK_ERR_TIMEOUT) ? ARM_FAULT_SRC_TIMEOUT : ARM_FAULT_SRC_TASK;
    }
    g_arm_task_status = status;
    g_arm_task_state = ARM_TASK_FAULT;
    so101_arm_enable_torque(&s_arm, 0U);
}

static uint8_t arm_task_fault_confirmed(uint8_t active,
                                        uint32_t *start_ms,
                                        uint32_t now,
                                        uint32_t confirm_ms)
{
    if (active == 0U) {
        *start_ms = 0UL;
        return 0U;
    }
    if (*start_ms == 0UL) {
        *start_ms = now;
        return 0U;
    }
    return ((now - *start_ms) >= confirm_ms) ? 1U : 0U;
}

static uint8_t arm_task_has_fatal_fault(void)
{
    uint32_t now = sts_servo_platform_get_tick_ms();
    uint32_t faults = arm_task_collect_faults();

    if (arm_task_fault_confirmed((faults & STS_SERVO_FAULT_OVER_TEMP) != 0UL,
                                 &s_over_temp_start_ms,
                                 now,
                                 ARM_TASK_OVER_TEMP_CONFIRM_MS) != 0U) {
        s_arm_fault_source = ARM_FAULT_SRC_SERVO;
        return 1U;
    }

    if (arm_task_fault_confirmed((faults & ARM_TASK_VOLTAGE_FAULT_MASK) != 0UL,
                                 &s_voltage_fault_start_ms,
                                 now,
                                 ARM_TASK_VOLTAGE_CONFIRM_MS) != 0U) {
        s_arm_fault_source = ARM_FAULT_SRC_SERVO;
        return 1U;
    }

    if (arm_task_fault_confirmed((faults & STS_SERVO_FAULT_BUS_TIMEOUT) != 0UL,
                                 &s_bus_timeout_start_ms,
                                 now,
                                 ARM_TASK_BUS_TIMEOUT_CONFIRM_MS) != 0U) {
        s_arm_fault_source = ARM_FAULT_SRC_SERVO_BUS;
        return 1U;
    }

    if (arm_task_fault_confirmed((faults & STS_SERVO_FAULT_STATUS_ERROR) != 0UL,
                                 &s_status_error_start_ms,
                                 now,
                                 ARM_TASK_STATUS_ERROR_CONFIRM_MS) != 0U) {
        s_arm_fault_source = ARM_FAULT_SRC_SERVO;
        return 1U;
    }

    if (s_consecutive_timeout_count > ARM_TASK_FATAL_TIMEOUT_COUNT) {
        s_arm_fault_source = ARM_FAULT_SRC_SERVO_BUS;
        return 1U;
    }

    /* Avoid stale module fault latches after feedback recovers. */
    s_arm.fatal_fault_flags = 0UL;
    return 0U;
}

static uint8_t arm_task_should_enter_fault(so101_arm_status_t mdl_status)
{
    if (arm_task_has_fatal_fault() != 0U) {
        return 1U;
    }

    if (mdl_status == SO101_ARM_ERR_FAULT) {
        /* The module may latch a transient fault before task-level confirmation. */
        s_arm.state = SO101_ARM_STATE_MOVING;
        s_arm.fatal_fault_flags = 0UL;
        return 0U;
    }

    return 0U;
}

static uint8_t arm_task_arm_reached(void)
{
    return (so101_arm_is_reached(&s_arm) != 0U) ? 1U : 0U;
}

static float arm_task_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t arm_task_gripper_reached(void)
{
    float current = so101_arm_get_gripper_angle(&s_arm);

    return (arm_task_absf(s_arm_target_gripper_rad - current) <=
            ARM_TASK_GRIPPER_REACHED_EPS_RAD)
               ? 1U
               : 0U;
}

static void arm_task_prepare_home_feedback_wait(void)
{
    for (uint8_t i = 0U; i < SO101_ARM_SERVO_COUNT; i++) {
        s_servo[i].feedback_dirty = 0U;
        s_servo[i].target_dirty = 0U;
    }
}

static uint8_t arm_task_active_joint_feedback_ready(void)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (s_servo[i].feedback_dirty == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static void arm_task_clear_active_joint_feedback(void)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        s_servo[i].feedback_dirty = 0U;
    }
}

static uint8_t arm_task_active_joint_targets_clean(void)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (s_servo[i].target_dirty != 0U) {
            return 0U;
        }
    }
    return 1U;
}

static void arm_task_sync_command_to_feedback(void)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        s_arm.current_joint_rad[i] = sts_servo_get_angle_rad(&s_servo[i]);
        s_arm.command_joint_rad[i] = s_arm.current_joint_rad[i];
    }
}

static void arm_task_set_targets_to_feedback(void)
{
    for (uint8_t i = 0U; i < SO101_ARM_SERVO_COUNT; i++) {
        sts_servo_set_angle_rad(&s_servo[i], s_servo[i].current_angle_rad);
    }
    s_arm_target_gripper_rad = so101_arm_get_gripper_angle(&s_arm);
}

static uint8_t arm_task_command_reached_target(void)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (arm_task_absf(s_arm.target_joint_rad[i] - s_arm.command_joint_rad[i]) >
            SO101_ARM_REACHED_ANGLE_EPS_RAD) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t arm_task_home_reached(void)
{
    if (arm_task_active_joint_feedback_ready() == 0U) {
        return 0U;
    }
    if (arm_task_command_reached_target() == 0U) {
        return 0U;
    }
    return arm_task_arm_reached();
}

static uint8_t arm_task_active_reached(void)
{
    if (s_arm_active_cmd.valid == 0U) {
        return 0U;
    }
    if (s_arm_active_cmd.cmd == ARM_CMD_GRIPPER) {
        return arm_task_gripper_reached();
    }
    if (s_arm_active_cmd.cmd == ARM_CMD_HOME) {
        return ((arm_task_arm_reached() != 0U) &&
                (arm_task_gripper_reached() != 0U))
                   ? 1U
                   : 0U;
    }
    return arm_task_arm_reached();
}

static uint8_t arm_task_runtime_reached(void)
{
    if ((g_arm_task_state == ARM_TASK_REACHED) &&
        (g_arm_task_status == ARM_TASK_OK)) {
        return 1U;
    }
    if (s_arm_active_cmd.valid != 0U) {
        return arm_task_active_reached();
    }
    return arm_task_arm_reached();
}

static arm_diag_code_t task_arm_result_diag(arm_result_t result,
                                            arm_reject_reason_t reject_reason,
                                            arm_fault_source_t fault_source)
{
    if (fault_source == ARM_FAULT_SRC_SERVO) {
        return ARM_DIAG_SERVO_FAULT;
    }
    if (fault_source == ARM_FAULT_SRC_SERVO_BUS) {
        return ARM_DIAG_SERVO_BUS_TIMEOUT;
    }
    if (fault_source == ARM_FAULT_SRC_TIMEOUT) {
        return ARM_DIAG_TASK_TIMEOUT;
    }
    if (reject_reason == ARM_REJECT_IK_UNREACHABLE) {
        return ARM_DIAG_IK_UNREACHABLE;
    }
    if ((reject_reason == ARM_REJECT_PARAM) ||
        (reject_reason == ARM_REJECT_TARGET_OUT_OF_RANGE)) {
        return ARM_DIAG_PARAM_ERROR;
    }
    if ((result == ARM_RESULT_REJECTED) ||
        (reject_reason == ARM_REJECT_BUSY) ||
        (reject_reason == ARM_REJECT_OWNER)) {
        return ARM_DIAG_BUSY;
    }
    return ARM_DIAG_OK;
}

static void task_arm_send_result(uint8_t seq,
                                 uint8_t cmd,
                                 uint8_t source,
                                 arm_result_t result,
                                 arm_reject_reason_t reject_reason,
                                 arm_fault_source_t fault_source,
                                 float requested_x,
                                 float requested_y,
                                 float requested_z,
                                 float requested_gripper_rad,
                                 float accepted_x,
                                 float accepted_y,
                                 float accepted_z,
                                 float accepted_gripper_rad,
                                 uint32_t now)
{
    ctrl_arm_result_payload_t payload = {0};

    payload.tick_ms = now;
    payload.request_seq = seq;
    payload.request_cmd = cmd;
    payload.request_source = source;
    payload.result = (uint8_t)result;
    payload.reject_reason = (uint8_t)reject_reason;
    payload.fault_source = (uint8_t)fault_source;
    payload.state_after = (uint8_t)g_arm_task_state;
    payload.diag_code = (uint8_t)task_arm_result_diag(result, reject_reason,
                                                      fault_source);
    payload.requested_x = requested_x;
    payload.requested_y = requested_y;
    payload.requested_z = requested_z;
    payload.requested_gripper_rad = requested_gripper_rad;
    payload.accepted_x = accepted_x;
    payload.accepted_y = accepted_y;
    payload.accepted_z = accepted_z;
    payload.accepted_gripper_rad = accepted_gripper_rad;
    payload.current_x = s_arm.current_position.x;
    payload.current_y = s_arm.current_position.y;
    payload.current_z = s_arm.current_position.z;
    payload.current_gripper_rad = so101_arm_get_gripper_angle(&s_arm);

    (void)telemetry_submit_result(CTRL_TARGET_ARM, CTRL_ARM_RPT_RESULT,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
}

static void task_arm_clear_active_cmd(void)
{
    s_arm_active_cmd.valid = 0U;
    s_arm_active_cmd.seq = 0U;
    s_arm_active_cmd.cmd = ARM_CMD_STOP;
    s_arm_active_cmd.source = CTRL_SRC_NONE;
    s_arm_active_cmd.requested_x = 0.0f;
    s_arm_active_cmd.requested_y = 0.0f;
    s_arm_active_cmd.requested_z = 0.0f;
    s_arm_active_cmd.requested_gripper_rad = 0.0f;
    s_arm_active_cmd.accepted_x = 0.0f;
    s_arm_active_cmd.accepted_y = 0.0f;
    s_arm_active_cmd.accepted_z = 0.0f;
    s_arm_active_cmd.accepted_gripper_rad = 0.0f;
}

static void task_arm_set_active_cmd(const arm_cmd_msg_t *msg,
                                    float accepted_x,
                                    float accepted_y,
                                    float accepted_z,
                                    float accepted_gripper_rad)
{
    if (msg == 0) {
        return;
    }

    s_arm_active_cmd.valid = 1U;
    s_arm_active_cmd.seq = msg->seq;
    s_arm_active_cmd.cmd = msg->type;
    s_arm_active_cmd.source = msg->source;
    s_arm_active_cmd.requested_x = msg->x;
    s_arm_active_cmd.requested_y = msg->y;
    s_arm_active_cmd.requested_z = msg->z;
    s_arm_active_cmd.requested_gripper_rad = msg->gripper_rad;
    s_arm_active_cmd.accepted_x = accepted_x;
    s_arm_active_cmd.accepted_y = accepted_y;
    s_arm_active_cmd.accepted_z = accepted_z;
    s_arm_active_cmd.accepted_gripper_rad = accepted_gripper_rad;
}

static void task_arm_send_active_result(arm_result_t result,
                                        arm_reject_reason_t reject_reason,
                                        arm_fault_source_t fault_source,
                                        uint32_t now)
{
    if (s_arm_active_cmd.valid == 0U) {
        return;
    }

    task_arm_send_result(s_arm_active_cmd.seq,
                         s_arm_active_cmd.cmd,
                         s_arm_active_cmd.source,
                         result,
                         reject_reason,
                         fault_source,
                         s_arm_active_cmd.requested_x,
                         s_arm_active_cmd.requested_y,
                         s_arm_active_cmd.requested_z,
                         s_arm_active_cmd.requested_gripper_rad,
                         s_arm_active_cmd.accepted_x,
                         s_arm_active_cmd.accepted_y,
                         s_arm_active_cmd.accepted_z,
                         s_arm_active_cmd.accepted_gripper_rad,
                         now);

    if (result != ARM_RESULT_ACCEPTED) {
        task_arm_clear_active_cmd();
    }
}

static void task_arm_supersede_active_cmd(uint32_t now)
{
    task_arm_send_active_result(ARM_RESULT_SUPERSEDED,
                                ARM_REJECT_NONE,
                                ARM_FAULT_SRC_NONE,
                                now);
}

static void arm_task_hold_pose_as_state(arm_task_state_t state, uint32_t now)
{
    float joint_rad[SO101_ACTIVE_JOINT_COUNT];
    float gripper_rad = so101_arm_get_gripper_angle(&s_arm);

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        joint_rad[i] = s_arm.current_joint_rad[i];
    }

    (void)so101_arm_set_joint_angles(&s_arm, joint_rad);
    memcpy(s_arm.command_joint_rad, joint_rad, sizeof(s_arm.command_joint_rad));
    s_arm.target_position = s_arm.current_position;
    s_arm.target_pose.x = s_arm.current_position.x;
    s_arm.target_pose.y = s_arm.current_position.y;
    s_arm.target_pose.z = s_arm.current_position.z;
    s_arm.target_pose.roll = s_arm.current_joint_rad[SO101_JOINT_WRIST_ROLL];
    s_arm.target_pose.pitch = 0.0f;
    so101_arm_set_gripper_angle(&s_arm, gripper_rad);
    s_arm_target_gripper_rad = gripper_rad;
    g_arm_task_status = ARM_TASK_OK;
    arm_task_change_state(state, now);
}

static void arm_task_hold_current_pose(uint32_t now)
{
    arm_task_hold_pose_as_state(ARM_TASK_WAIT_TARGET, now);
}

static arm_reject_reason_t task_arm_reach_status_to_reject(so101_arm_reach_status_t status)
{
    switch (status) {
    case SO101_ARM_REACH_OK:
        return ARM_REJECT_NONE;
    case SO101_ARM_REACH_ERR_XYZ_RANGE:
    case SO101_ARM_REACH_ERR_Z_LOW:
    case SO101_ARM_REACH_ERR_Z_HIGH:
    case SO101_ARM_REACH_ERR_LIMIT_MARGIN:
        return ARM_REJECT_TARGET_OUT_OF_RANGE;
    case SO101_ARM_REACH_ERR_NULL:
        return ARM_REJECT_PARAM;
    case SO101_ARM_REACH_ERR_IK:
    default:
        return ARM_REJECT_IK_UNREACHABLE;
    }
}

static void task_arm_force_stop(uint32_t now)
{
    arm_task_hold_pose_as_state(ARM_TASK_REACHED, now);
}

static uint8_t task_arm_can_accept_control(const arm_cmd_msg_t *msg)
{
    if (msg == 0) {
        return 0U;
    }

    return control_arbitration_can_accept(&arm_arbitration,
                                          (control_source_e)msg->source);
}

static void task_arm_accept_control(const arm_cmd_msg_t *msg, uint32_t now)
{
    if (msg != 0) {
        control_arbitration_accept(&arm_arbitration,
                                   (control_source_e)msg->source, now);
    }
}

static void task_arm_clear_fault_direct(uint32_t now)
{
    if ((g_arm_task_state == ARM_TASK_FAULT) ||
        (g_arm_task_state == ARM_TASK_TORQUE_DISABLED)) {
        g_arm_task_status = ARM_TASK_BUSY;
        s_consecutive_timeout_count = 0UL;
        s_over_temp_start_ms = 0UL;
        s_voltage_fault_start_ms = 0UL;
        s_bus_timeout_start_ms = 0UL;
        s_status_error_start_ms = 0UL;
        s_arm_fault_source = ARM_FAULT_SRC_NONE;
        arm_task_change_state(ARM_TASK_ENABLE_TORQUE, now);
    }
}

static void task_arm_reject_msg(const arm_cmd_msg_t *msg,
                                arm_reject_reason_t reason,
                                uint32_t now)
{
    if (msg == 0) {
        return;
    }

    task_arm_send_result(msg->seq,
                         msg->type,
                         msg->source,
                         ARM_RESULT_REJECTED,
                         reason,
                         ARM_FAULT_SRC_NONE,
                         msg->x,
                         msg->y,
                         msg->z,
                         msg->gripper_rad,
                         s_arm.target_position.x,
                         s_arm.target_position.y,
                         s_arm.target_position.z,
                         s_arm_target_gripper_rad,
                         now);
}

static uint8_t task_arm_value_is_invalid(float value)
{
    return (value != value) ? 1U : 0U;
}

static arm_reject_reason_t task_arm_validate_common_xyz(float x, float y, float z)
{
    if ((task_arm_value_is_invalid(x) != 0U) ||
        (task_arm_value_is_invalid(y) != 0U) ||
        (task_arm_value_is_invalid(z) != 0U)) {
        return ARM_REJECT_PARAM;
    }
    return ARM_REJECT_NONE;
}

static arm_reject_reason_t task_arm_apply_xyz_target(const arm_cmd_msg_t *msg)
{
    so101_arm_position_t position;
    so101_arm_reach_result_t reach_result;
    so101_arm_reach_status_t reach_status;
    arm_reject_reason_t reject_reason;

    if (msg == 0) {
        return ARM_REJECT_PARAM;
    }

    reject_reason = task_arm_validate_common_xyz(msg->x, msg->y, msg->z);
    if (reject_reason != ARM_REJECT_NONE) {
        return reject_reason;
    }

    position.x = msg->x;
    position.y = msg->y;
    position.z = msg->z;
    reach_status = so101_arm_check_reachable(&s_arm, &position, &reach_result);
    if (reach_status != SO101_ARM_REACH_OK) {
        s_arm.ik_info.status = ROBOT_ERR_UNREACHABLE;
        s_arm.ik_info.position_error_m = reach_result.position_error_m;
        s_arm.ik_info.pitch_error_rad = reach_result.pitch_error_rad;
        s_arm.ik_info.joint_limit_margin_rad = reach_result.limit_margin_rad;
        s_arm.ik_info.side_error_m = reach_result.side_error_m;
        s_arm.ik_info.iterations = reach_result.ik_iterations;
        return task_arm_reach_status_to_reject(reach_status);
    }

    (void)so101_arm_set_joint_angles(&s_arm, reach_result.joint_rad);
    s_arm.target_position = position;
    s_arm.target_pose.x = position.x;
    s_arm.target_pose.y = position.y;
    s_arm.target_pose.z = position.z;
    return ARM_REJECT_NONE;
}

static arm_reject_reason_t task_arm_apply_pose_target(const arm_cmd_msg_t *msg)
{
    robot_pose_target_t pose_target;
    float joint_rad[SO101_ACTIVE_JOINT_COUNT];
    robot_status_t robot_status;
    arm_reject_reason_t reject_reason;

    if (msg == 0) {
        return ARM_REJECT_PARAM;
    }

    reject_reason = task_arm_validate_common_xyz(msg->x, msg->y, msg->z);
    if ((reject_reason != ARM_REJECT_NONE) ||
        (task_arm_value_is_invalid(msg->roll) != 0U) ||
        (task_arm_value_is_invalid(msg->pitch) != 0U)) {
        return (reject_reason != ARM_REJECT_NONE) ? reject_reason : ARM_REJECT_PARAM;
    }

    pose_target.position.x = msg->x;
    pose_target.position.y = msg->y;
    pose_target.position.z = msg->z;
    pose_target.roll_rad = msg->roll;
    pose_target.pitch_rad = msg->pitch;
    robot_status = so101_pose_ik(&pose_target,
                                 s_arm.current_joint_rad,
                                 &s_arm.ik_options,
                                 joint_rad,
                                 &s_arm.ik_info);
    s_arm.last_robot_status = robot_status;
    if (robot_status != ROBOT_OK) {
        return ARM_REJECT_IK_UNREACHABLE;
    }

    (void)so101_arm_set_joint_angles(&s_arm, joint_rad);
    s_arm.target_position.x = msg->x;
    s_arm.target_position.y = msg->y;
    s_arm.target_position.z = msg->z;
    s_arm.target_pose.x = msg->x;
    s_arm.target_pose.y = msg->y;
    s_arm.target_pose.z = msg->z;
    s_arm.target_pose.roll = msg->roll;
    s_arm.target_pose.pitch = msg->pitch;
    return ARM_REJECT_NONE;
}

static arm_reject_reason_t task_arm_apply_home_target(void)
{
    float joint_rad[SO101_ACTIVE_JOINT_COUNT] = {
        SO101_ARM_HOME_JOINT_1_RAD,
        SO101_ARM_HOME_JOINT_2_RAD,
        SO101_ARM_HOME_JOINT_3_RAD,
        SO101_ARM_HOME_JOINT_4_RAD,
        SO101_ARM_HOME_JOINT_5_RAD,
    };
    mat4f_t tip_t;

    (void)so101_arm_set_joint_angles(&s_arm, joint_rad);
    if (so101_fk_compute(joint_rad, &tip_t) == ROBOT_OK) {
        s_arm.target_position.x = tip_t.m[0][3];
        s_arm.target_position.y = tip_t.m[1][3];
        s_arm.target_position.z = tip_t.m[2][3];
    }
    so101_arm_set_gripper_angle(&s_arm, SO101_ARM_HOME_GRIPPER_RAD);
    s_arm_target_gripper_rad = SO101_ARM_HOME_GRIPPER_RAD;
    return ARM_REJECT_NONE;
}

static arm_reject_reason_t task_arm_apply_gripper_target(const arm_cmd_msg_t *msg,
                                                         uint32_t now)
{
    if ((msg == 0) || (task_arm_value_is_invalid(msg->gripper_rad) != 0U)) {
        return ARM_REJECT_PARAM;
    }

    arm_task_hold_pose_as_state(ARM_TASK_EXECUTE, now);
    so101_arm_set_gripper_angle(&s_arm, msg->gripper_rad);
    s_arm_target_gripper_rad = msg->gripper_rad;
    return ARM_REJECT_NONE;
}

static void task_arm_accept_latest_target(const arm_cmd_msg_t *msg, uint32_t now)
{
    arm_reject_reason_t reject_reason = ARM_REJECT_NONE;
    float accepted_x;
    float accepted_y;
    float accepted_z;
    float accepted_gripper_rad;

    if (msg == 0) {
        return;
    }

    if (g_arm_task_state == ARM_TASK_FAULT) {
        task_arm_reject_msg(msg, ARM_REJECT_IN_FAULT, now);
        return;
    }
    if (g_arm_task_state == ARM_TASK_TORQUE_DISABLED) {
        task_arm_reject_msg(msg, ARM_REJECT_TORQUE_DISABLED, now);
        return;
    }
    if (arm_task_can_accept_motion_command() == 0U) {
        task_arm_reject_msg(msg, ARM_REJECT_BUSY, now);
        return;
    }
    if (task_arm_can_accept_control(msg) == 0U) {
        task_arm_reject_msg(msg, ARM_REJECT_OWNER, now);
        return;
    }

    task_arm_supersede_active_cmd(now);
    s_arm_fault_source = ARM_FAULT_SRC_NONE;

    switch (msg->type) {
    case ARM_CMD_HOME:
        reject_reason = task_arm_apply_home_target();
        break;

    case ARM_CMD_MOVE_XYZ:
        reject_reason = task_arm_apply_xyz_target(msg);
        break;

    case ARM_CMD_MOVE_POSE:
        reject_reason = task_arm_apply_pose_target(msg);
        break;

    case ARM_CMD_GRIPPER:
        reject_reason = task_arm_apply_gripper_target(msg, now);
        break;

    default:
        reject_reason = ARM_REJECT_UNKNOWN_CMD;
        break;
    }

    if (reject_reason != ARM_REJECT_NONE) {
        if (reject_reason == ARM_REJECT_IK_UNREACHABLE) {
            s_arm_fault_source = ARM_FAULT_SRC_IK;
        }
        arm_task_hold_current_pose(now);
        task_arm_reject_msg(msg, reject_reason, now);
        control_arbitration_release(&arm_arbitration);
        return;
    }

    g_arm_task_status = ARM_TASK_BUSY;
    arm_task_change_state(ARM_TASK_EXECUTE, now);

    accepted_x = s_arm.target_position.x;
    accepted_y = s_arm.target_position.y;
    accepted_z = s_arm.target_position.z;
    accepted_gripper_rad = s_arm_target_gripper_rad;
    task_arm_set_active_cmd(msg, accepted_x, accepted_y, accepted_z,
                            accepted_gripper_rad);
    task_arm_send_active_result(ARM_RESULT_ACCEPTED,
                                ARM_REJECT_NONE,
                                ARM_FAULT_SRC_NONE,
                                now);
    task_arm_accept_control(msg, now);
}

static void task_arm_handle_msg(const arm_cmd_msg_t *msg, uint32_t now)
{
    if (msg == 0) {
        return;
    }

    switch (msg->type) {
    case ARM_CMD_STOP:
        /* STOP is always accepted and clears module ownership. */
        if (g_arm_task_state != ARM_TASK_FAULT) {
            arm_task_hold_pose_as_state(ARM_TASK_REACHED, now);
        }
        task_arm_send_active_result((g_arm_task_state == ARM_TASK_FAULT) ? ARM_RESULT_FAILED : ARM_RESULT_ABORTED,
                                    ARM_REJECT_NONE,
                                    s_arm_fault_source,
                                    now);
        control_arbitration_release(&arm_arbitration);
        task_arm_send_result(msg->seq,
                             msg->type,
                             msg->source,
                             ARM_RESULT_COMPLETED,
                             ARM_REJECT_NONE,
                             ARM_FAULT_SRC_NONE,
                             msg->x,
                             msg->y,
                             msg->z,
                             msg->gripper_rad,
                             s_arm.target_position.x,
                             s_arm.target_position.y,
                             s_arm.target_position.z,
                             s_arm_target_gripper_rad,
                             now);
        break;

    case ARM_CMD_HOME:
    case ARM_CMD_MOVE_XYZ:
    case ARM_CMD_MOVE_POSE:
    case ARM_CMD_GRIPPER:
        task_arm_accept_latest_target(msg, now);
        break;

    case ARM_CMD_CLEAR_FAULT:
        /* Fault recovery does not take ownership from the active motion source. */
        task_arm_send_active_result((g_arm_task_state == ARM_TASK_FAULT) ? ARM_RESULT_FAILED : ARM_RESULT_ABORTED,
                                    ARM_REJECT_NONE,
                                    s_arm_fault_source,
                                    now);
        task_arm_clear_fault_direct(now);
        task_arm_send_result(msg->seq,
                             msg->type,
                             msg->source,
                             ARM_RESULT_COMPLETED,
                             ARM_REJECT_NONE,
                             ARM_FAULT_SRC_NONE,
                             msg->x,
                             msg->y,
                             msg->z,
                             msg->gripper_rad,
                             s_arm.target_position.x,
                             s_arm.target_position.y,
                             s_arm.target_position.z,
                             s_arm_target_gripper_rad,
                             now);
        break;

    case ARM_CMD_DISABLE_TORQUE:
        /* DISABLE_TORQUE is always accepted as a safety command. */
        task_arm_send_active_result(ARM_RESULT_ABORTED,
                                    ARM_REJECT_NONE,
                                    ARM_FAULT_SRC_NONE,
                                    now);
        so101_arm_enable_torque(&s_arm, 0U);
        g_arm_task_status = ARM_TASK_OK;
        arm_task_change_state(ARM_TASK_TORQUE_DISABLED, now);
        control_arbitration_release(&arm_arbitration);
        task_arm_send_result(msg->seq,
                             msg->type,
                             msg->source,
                             ARM_RESULT_COMPLETED,
                             ARM_REJECT_NONE,
                             ARM_FAULT_SRC_NONE,
                             msg->x,
                             msg->y,
                             msg->z,
                             msg->gripper_rad,
                             s_arm.target_position.x,
                             s_arm.target_position.y,
                             s_arm.target_position.z,
                             s_arm_target_gripper_rad,
                             now);
        break;

    default:
        task_arm_reject_msg(msg, ARM_REJECT_UNKNOWN_CMD, now);
        break;
    }
}

static void task_arm_drain_queue(uint32_t now)
{
    arm_cmd_msg_t msg;

    /* Drain all queued commands so stale target packets do not lag behind. */
    while (osMessageQueueGet(ArmCmdQueueHandle, &msg, 0U, 0U) == osOK) {
        task_arm_handle_msg(&msg, now);
    }
}

static void task_arm_update_arbitration(uint32_t now)
{
    if (control_arbitration_is_timeout(&arm_arbitration, now,
                                       ARM_CONTROL_TIMEOUT_MS) != 0U) {
        /* Fail safe: hold the current pose if the active controller disappears. */
        s_arm_fault_source = ARM_FAULT_SRC_TIMEOUT;
        task_arm_force_stop(now);
        task_arm_send_active_result(ARM_RESULT_FAILED,
                                    ARM_REJECT_NONE,
                                    ARM_FAULT_SRC_TIMEOUT,
                                    now);
        control_arbitration_release(&arm_arbitration);
    }

    if ((g_arm_task_state != ARM_TASK_TORQUE_DISABLED) &&
        (g_arm_task_status == ARM_TASK_OK) &&
        ((g_arm_task_state == ARM_TASK_IDLE) ||
         (g_arm_task_state == ARM_TASK_REACHED))) {
        control_arbitration_release(&arm_arbitration);
    }
}

static void task_arm_update_active_result(uint32_t now)
{
    if (s_arm_active_cmd.valid == 0U) {
        return;
    }

    if (g_arm_task_state == ARM_TASK_FAULT) {
        task_arm_send_active_result(ARM_RESULT_FAILED,
                                    ARM_REJECT_NONE,
                                    s_arm_fault_source,
                                    now);
    } else if (arm_task_active_reached() != 0U) {
        task_arm_send_active_result(ARM_RESULT_COMPLETED,
                                    ARM_REJECT_NONE,
                                    ARM_FAULT_SRC_NONE,
                                    now);
    }
}

static uint32_t arm_task_elapsed_since_or_zero(uint32_t now, uint32_t start)
{
    return (start == 0UL) ? 0UL : (uint32_t)(now - start);
}

static void arm_task_state_snapshot_update(uint32_t now)
{
    uint8_t fault_servo_index = ARM_TASK_FAULT_SERVO_NONE;
    arm_diag_code_t diag_code = ARM_DIAG_OK;
    arm_diag_severity_t severity = ARM_DIAG_SEV_INFO;
    uint32_t detail_u32 = 0UL;
    float detail_f32 = 0.0f;

    for (uint8_t i = 0U; i < SO101_ARM_SERVO_COUNT; i++) {
        if ((fault_servo_index == ARM_TASK_FAULT_SERVO_NONE) &&
            (s_servo[i].fault_flags != STS_SERVO_FAULT_NONE)) {
            fault_servo_index = i;
        }
    }

    /* Summary first: one field tells Watch/uplink where to drill down. */
    if (s_arm_fault_source == ARM_FAULT_SRC_SERVO) {
        diag_code = ARM_DIAG_SERVO_FAULT;
        severity = ARM_DIAG_SEV_FATAL;
        detail_u32 = (fault_servo_index < SO101_ARM_SERVO_COUNT) ? s_servo[fault_servo_index].fault_flags : s_arm.fault_flags;
    } else if (s_arm_fault_source == ARM_FAULT_SRC_SERVO_BUS) {
        diag_code = ARM_DIAG_SERVO_BUS_TIMEOUT;
        severity = ARM_DIAG_SEV_FATAL;
        detail_u32 = (uint32_t)s_servo_bus.last_status;
    } else if ((g_arm_task_status == ARM_TASK_ERR_TIMEOUT) ||
               (s_arm_fault_source == ARM_FAULT_SRC_TIMEOUT)) {
        diag_code = ARM_DIAG_TASK_TIMEOUT;
        severity = ARM_DIAG_SEV_ERROR;
        detail_u32 = 0UL;
    } else if ((s_arm_fault_source == ARM_FAULT_SRC_IK) ||
               (g_arm_task_status == ARM_TASK_ERR_IK)) {
        diag_code = ARM_DIAG_IK_UNREACHABLE;
        severity = ARM_DIAG_SEV_ERROR;
        detail_u32 = (uint32_t)s_arm.ik_info.status;
        detail_f32 = s_arm.ik_info.position_error_m;
    } else if ((s_arm_fault_source == ARM_FAULT_SRC_MDL) ||
               (g_arm_task_status == ARM_TASK_ERR_FAULT)) {
        diag_code = ARM_DIAG_MDL_FAULT;
        severity = ARM_DIAG_SEV_ERROR;
        detail_u32 = (uint32_t)s_last_mdl_status;
    } else if (g_arm_task_status == ARM_TASK_ERR_PARAM) {
        diag_code = ARM_DIAG_PARAM_ERROR;
        severity = ARM_DIAG_SEV_ERROR;
    } else if (g_arm_task_status == ARM_TASK_BUSY) {
        diag_code = ARM_DIAG_BUSY;
    }

    g_arm_state.summary.tick_ms = now;
    g_arm_state.summary.code = diag_code;
    g_arm_state.summary.severity = severity;
    g_arm_state.summary.fault_source = s_arm_fault_source;
    g_arm_state.summary.task_state = g_arm_task_state;
    g_arm_state.summary.task_status = g_arm_task_status;
    g_arm_state.summary.fault_servo_index = fault_servo_index;
    g_arm_state.summary.detail_u32 = detail_u32;
    g_arm_state.summary.detail_f32 = detail_f32;

    g_arm_state.runtime.tick_ms = now;
    g_arm_state.runtime.task_state = g_arm_task_state;
    g_arm_state.runtime.task_status = g_arm_task_status;
    g_arm_state.runtime.current_xyz[0] = s_arm.current_position.x;
    g_arm_state.runtime.current_xyz[1] = s_arm.current_position.y;
    g_arm_state.runtime.current_xyz[2] = s_arm.current_position.z;
    g_arm_state.runtime.target_xyz[0] = s_arm.target_position.x;
    g_arm_state.runtime.target_xyz[1] = s_arm.target_position.y;
    g_arm_state.runtime.target_xyz[2] = s_arm.target_position.z;
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        g_arm_state.runtime.current_joint_rad[i] = s_arm.current_joint_rad[i];
        g_arm_state.runtime.target_joint_rad[i] = s_arm.target_joint_rad[i];
    }
    g_arm_state.runtime.gripper_rad = so101_arm_get_gripper_angle(&s_arm);
    g_arm_state.runtime.is_reached = arm_task_runtime_reached();
    g_arm_state.runtime.is_busy = (g_arm_task_status == ARM_TASK_BUSY) ? 1U : 0U;
    g_arm_state.runtime.has_fault = ((g_arm_task_state == ARM_TASK_FAULT) ||
                                     (g_arm_task_status >= ARM_TASK_ERR_BUS))
                                        ? 1U
                                        : 0U;

    g_arm_state.detail.last_mdl_status = s_last_mdl_status;
    g_arm_state.detail.last_robot_status = s_arm.last_robot_status;
    g_arm_state.detail.ik_status = s_arm.ik_info.status;
    g_arm_state.detail.ik_iterations = s_arm.ik_info.iterations;
    g_arm_state.detail.ik_position_error_m = s_arm.ik_info.position_error_m;
    g_arm_state.detail.ik_pitch_error_rad = s_arm.ik_info.pitch_error_rad;
    g_arm_state.detail.ik_joint_limit_margin_rad = s_arm.ik_info.joint_limit_margin_rad;
    g_arm_state.detail.ik_side_error_m = s_arm.ik_info.side_error_m;
    g_arm_state.detail.fault_flags = s_arm.fault_flags;
    g_arm_state.detail.fatal_fault_flags = s_arm.fatal_fault_flags;
    g_arm_state.detail.consecutive_timeout_count = s_consecutive_timeout_count;
    g_arm_state.detail.over_temp_elapsed_ms = arm_task_elapsed_since_or_zero(now, s_over_temp_start_ms);
    g_arm_state.detail.voltage_fault_elapsed_ms = arm_task_elapsed_since_or_zero(now, s_voltage_fault_start_ms);
    g_arm_state.detail.bus_timeout_elapsed_ms = arm_task_elapsed_since_or_zero(now, s_bus_timeout_start_ms);
    g_arm_state.detail.status_error_elapsed_ms = arm_task_elapsed_since_or_zero(now, s_status_error_start_ms);
    for (uint8_t i = 0U; i < SO101_ARM_SERVO_COUNT; i++) {
        g_arm_state.detail.servo_fault_flags[i] = s_servo[i].fault_flags;
        g_arm_state.detail.servo_last_status[i] = s_servo[i].last_status;
        g_arm_state.detail.servo_voltage[i] = s_servo[i].current_voltage;
        g_arm_state.detail.servo_temperature[i] = s_servo[i].current_temperature;
        g_arm_state.detail.servo_load[i] = s_servo[i].current_load;
        g_arm_state.detail.servo_current[i] = s_servo[i].current_current;
    }
    g_arm_state.detail.bus_last_status = s_servo_bus.last_status;
    g_arm_state.detail.bus_parser_state = s_servo_bus.parser_state;
    g_arm_state.detail.bus_tx_busy = s_servo_bus.tx_busy;
    g_arm_state.detail.bus_queue_count = s_servo_bus.queue_count;
    g_arm_state.detail.bus_response_count = s_servo_bus.response_count;
    g_arm_state.detail.bus_pending_response_count = s_servo_bus.pending_response_count;
}

static void arm_task_send_status(void)
{
    ctrl_arm_status_payload_t payload = {0};
    uint32_t now = g_arm_state.summary.tick_ms;

    if (s_arm_status_phase_ready == 0U) {
        if (s_arm_status_phase_start_ms == 0UL) {
            s_arm_status_phase_start_ms = now;
        }
        if ((uint32_t)(now - s_arm_status_phase_start_ms) <
            ARM_STATUS_PHASE_MS) {
            return;
        }
        s_arm_status_phase_ready = 1U;
        s_arm_status_last_ms = now - ARM_STATUS_PERIOD_MS;
    }

    if ((uint32_t)(now - s_arm_status_last_ms) < ARM_STATUS_PERIOD_MS) {
        return;
    }
    s_arm_status_last_ms = now;

    payload.tick_ms = g_arm_state.summary.tick_ms;
    payload.state = (uint8_t)g_arm_state.runtime.task_state;
    payload.status = (uint8_t)g_arm_state.runtime.task_status;
    payload.is_busy = g_arm_state.runtime.is_busy;
    payload.has_fault = g_arm_state.runtime.has_fault;
    payload.active_cmd = (s_arm_active_cmd.valid != 0U) ? s_arm_active_cmd.cmd : ARM_CMD_STOP;
    payload.active_source = (s_arm_active_cmd.valid != 0U) ? s_arm_active_cmd.source : CTRL_SRC_NONE;
    payload.active_seq = (s_arm_active_cmd.valid != 0U) ? s_arm_active_cmd.seq : 0U;
    payload.diag_code = (uint8_t)g_arm_state.summary.code;
    payload.current_x = g_arm_state.runtime.current_xyz[0];
    payload.current_y = g_arm_state.runtime.current_xyz[1];
    payload.current_z = g_arm_state.runtime.current_xyz[2];
    payload.target_x = g_arm_state.runtime.target_xyz[0];
    payload.target_y = g_arm_state.runtime.target_xyz[1];
    payload.target_z = g_arm_state.runtime.target_xyz[2];
    payload.current_gripper_rad = g_arm_state.runtime.gripper_rad;
    payload.target_gripper_rad = s_arm_target_gripper_rad;
    payload.fault_flags = g_arm_state.detail.fault_flags;

    (void)telemetry_submit_status(CTRL_TARGET_ARM, CTRL_ARM_RPT_STATUS,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
}

void StartArmTask(void *argument)
{
    uint32_t now;
    so101_arm_status_t mdl_status = SO101_ARM_OK;

    (void)argument;

    control_arbitration_init(&arm_arbitration);

    for (;;) {
        now = sts_servo_platform_get_tick_ms();

        task_arm_drain_queue(now);

        switch (g_arm_task_state) {
        case ARM_TASK_INIT:
            so101_arm_init(&s_arm);
            g_arm_task_status = ARM_TASK_BUSY;
            mdl_status = SO101_ARM_OK;
            s_consecutive_timeout_count = 0UL;
            s_arm_fault_source = ARM_FAULT_SRC_NONE;
            s_arm_target_gripper_rad = so101_arm_get_gripper_angle(&s_arm);
            task_arm_clear_active_cmd();
            s_home_started = 0U;
            s_home_feedback_wait_started = 0U;
            s_home_hold_target_started = 0U;
            arm_task_change_state(ARM_TASK_ENABLE_TORQUE, now);
            break;

        case ARM_TASK_ENABLE_TORQUE:
            if (s_home_feedback_wait_started == 0U) {
                arm_task_prepare_home_feedback_wait();
                s_home_feedback_wait_started = 1U;
                s_home_hold_target_started = 0U;
            }

            arm_servo_group_update(&s_servo_group);
            if (arm_task_active_joint_feedback_ready() == 0U) {
                break;
            }

            if (s_home_hold_target_started == 0U) {
                arm_task_sync_command_to_feedback();
                arm_task_set_targets_to_feedback();
                s_home_hold_target_started = 1U;
                break;
            }

            if (arm_task_active_joint_targets_clean() == 0U) {
                break;
            }

            so101_arm_enable_torque(&s_arm, 1U);
            s_consecutive_timeout_count = 0UL;
            s_bus_timeout_start_ms = 0UL;
            s_home_feedback_wait_started = 0U;
            s_home_hold_target_started = 0U;
            arm_task_change_state(ARM_TASK_HOME, now);
            break;

        case ARM_TASK_HOME: {
            arm_reject_reason_t home_reject;

            if (s_home_started == 0U) {
                home_reject = task_arm_apply_home_target();
                if (home_reject != ARM_REJECT_NONE) {
                    s_arm_fault_source = (home_reject == ARM_REJECT_IK_UNREACHABLE) ? ARM_FAULT_SRC_IK : ARM_FAULT_SRC_TASK;
                    arm_task_enter_fault((home_reject == ARM_REJECT_IK_UNREACHABLE) ? ARM_TASK_ERR_IK : ARM_TASK_ERR_PARAM);
                    break;
                }
                arm_task_clear_active_joint_feedback();
                s_home_started = 1U;
            }

            mdl_status = so101_arm_update(&s_arm, ARM_TASK_DT_S);
            if (arm_task_has_fatal_fault() != 0U) {
                arm_task_enter_fault(ARM_TASK_ERR_FAULT);
            } else if (arm_task_home_reached() != 0U) {
                g_arm_task_status = ARM_TASK_OK;
                s_home_started = 0U;
                s_home_feedback_wait_started = 0U;
                s_home_hold_target_started = 0U;
                arm_task_change_state(ARM_TASK_IDLE, now);
            } else if (arm_elapsed_ms(now, s_state_start_ms) > ARM_TASK_HOME_TIMEOUT_MS) {
                arm_task_enter_fault(ARM_TASK_ERR_TIMEOUT);
            }
            break;
        }

        case ARM_TASK_IDLE:
            mdl_status = so101_arm_update(&s_arm, ARM_TASK_DT_S);
            if (arm_task_has_fatal_fault() != 0U) {
                arm_task_enter_fault(ARM_TASK_ERR_FAULT);
            }
            break;

        case ARM_TASK_EXECUTE:
            mdl_status = so101_arm_update(&s_arm, ARM_TASK_DT_S);
            if (mdl_status == SO101_ARM_ERR_IK) {
                s_arm_fault_source = ARM_FAULT_SRC_IK;
                arm_task_hold_current_pose(now);
                task_arm_send_active_result(ARM_RESULT_REJECTED,
                                            ARM_REJECT_IK_UNREACHABLE,
                                            ARM_FAULT_SRC_NONE,
                                            now);
            } else if (arm_task_should_enter_fault(mdl_status) != 0U) {
                if (s_arm_fault_source == ARM_FAULT_SRC_NONE) {
                    s_arm_fault_source = ARM_FAULT_SRC_MDL;
                }
                arm_task_enter_fault(ARM_TASK_ERR_FAULT);
            } else if (arm_task_active_reached() != 0U) {
                g_arm_task_status = ARM_TASK_OK;
                arm_task_change_state(ARM_TASK_REACHED, now);
            }
            break;

        case ARM_TASK_WAIT_TARGET:
            mdl_status = so101_arm_update(&s_arm, ARM_TASK_DT_S);
            if (mdl_status == SO101_ARM_ERR_IK) {
                s_arm_fault_source = ARM_FAULT_SRC_IK;
                arm_task_hold_current_pose(now);
            } else if (arm_task_should_enter_fault(mdl_status) != 0U) {
                if (s_arm_fault_source == ARM_FAULT_SRC_NONE) {
                    s_arm_fault_source = ARM_FAULT_SRC_MDL;
                }
                arm_task_enter_fault(ARM_TASK_ERR_FAULT);
            }
            break;

        case ARM_TASK_REACHED:
            mdl_status = so101_arm_update(&s_arm, ARM_TASK_DT_S);
            if (arm_task_has_fatal_fault() != 0U) {
                arm_task_enter_fault(ARM_TASK_ERR_FAULT);
            }
            break;

        case ARM_TASK_TORQUE_DISABLED:
        case ARM_TASK_FAULT:
        default:
            arm_servo_group_update(&s_servo_group);
            break;
        }

        task_arm_update_arbitration(now);
        task_arm_update_active_result(now);
        s_last_mdl_status = mdl_status;
        arm_task_state_snapshot_update(now);
        arm_task_send_status();

        osDelay(ARM_TASK_PERIOD_MS);
    }
}
