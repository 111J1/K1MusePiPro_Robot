#include "task_lift.h"

#include "dev_brush_motor.h"
#include "dev_lift_home_sensor.h"
#include "drv_brush_motor.h"
#include "drv_lift_home_sensor.h"
#include "mdl_control_arbitration.h"
#include "mdl_lift.h"
#include "mdl_lift_config.h"
#include "task_telemetry.h"

#define LIFT_TASK_PERIOD_MS (10U)
#define LIFT_STATUS_PERIOD_MS (50U)
#define LIFT_STATUS_PHASE_MS (40U)
#define LIFT_TASK_DT (0.01f)
#define LIFT_CONTROL_TIMEOUT_MS (10000U)

typedef enum {
    LIFT_DEBUG_MOVE_IDLE = 0,
    LIFT_DEBUG_MOVE_ACCEPTED,
    LIFT_DEBUG_MOVE_NOT_READY,
    LIFT_DEBUG_MOVE_BUSY,
    LIFT_DEBUG_MOVE_OUT_OF_RANGE,
} lift_debug_move_status_e;

typedef struct {
    uint8_t valid;
    uint8_t seq;
    uint8_t cmd;
    uint8_t source;
    float requested_z;
    float accepted_z;
} lift_active_cmd_t;

brush_motor_t lift_motor = {
    .pid = {
        .kp = 0.5f,
        .ki = 0.1f,
        .kd = 0.0f,
        .out_min = -100.0f,
        .out_max = 100.0f,
    },
    .max_rpm = 330.0f,
    .block_max_cnt = 50U,
    .one_round_encoder_count = LIFT_ENCODER_COUNT_PER_ROUND,
    .driver = &lift_motor_driver,
};

static lift_home_sensor_device_t lift_bottom_sensor = {
    .driver = &lift_bottom_sensor_driver,
};

static lift_home_sensor_device_t lift_middle_sensor = {
    .driver = &lift_middle_sensor_driver,
};

static lift_home_sensor_device_t lift_home_middle_sensor = {
    .driver = &lift_middle_sensor_driver,
};

static lift_home_sensor_device_t lift_top_sensor = {
    .driver = &lift_top_sensor_driver,
};

static lift_t lift = {
    .z_min = LIFT_Z_MIN_M,
    .z_max = LIFT_Z_MAX_M,
    .max_v = LIFT_MAX_V_MPS,
    .reached_eps = LIFT_REACHED_EPS_M,
    .position_kp = LIFT_POSITION_KP,
    .motor = {
        .ctx = &lift_motor,
        .init = brush_motor_init,
        .set_rpm = brush_motor_set_rpm,
        .get_rpm = brush_motor_get_rpm,
        .get_total_encoder_count = brush_motor_get_total_encoder_count,
        .reset_total_encoder_count = brush_motor_reset_total_encoder_count,
        .update = brush_motor_update,
    },
    .home_sensor = {
        .ctx = &lift_home_middle_sensor,
        .init = lift_home_sensor_init,
        .update = lift_home_sensor_update,
        .get_level = lift_home_sensor_get_level,
        .get_last_level = lift_home_sensor_get_last_level,
        .get_rising_edge = lift_home_sensor_get_rising_edge,
        .get_falling_edge = lift_home_sensor_get_falling_edge,
        .is_initialized = lift_home_sensor_is_initialized,
    },
};

static module_control_arbitration_t lift_arbitration;
static uint32_t lift_home_start_tick = 0U;
static lift_fault_reason_t s_lift_fault_reason = LIFT_FAULT_NONE;
static lift_reject_reason_t s_lift_reject_reason = LIFT_REJECT_NONE;
static float s_lift_rejected_target_z = 0.0f;
static uint32_t s_lift_status_phase_start_ms = 0U;
static uint32_t s_lift_status_last_ms = 0U;
static uint8_t s_lift_status_phase_ready = 0U;
static lift_active_cmd_t s_lift_active_cmd;
volatile lift_state_snapshot_t g_lift_state;
volatile float g_lift_debug_move_z = 0.0f;
volatile float g_lift_debug_move_target_z = 0.0f;
volatile uint8_t g_lift_debug_move_trigger = 0U;
volatile uint8_t g_lift_debug_move_status = LIFT_DEBUG_MOVE_IDLE;

static void task_lift_stop(void);
static void task_lift_abort_motion(void);
static uint8_t task_lift_can_move_z(void);

static uint32_t task_lift_tick_to_ms(uint32_t tick)
{
    uint32_t tick_freq = osKernelGetTickFreq();

    if (tick_freq == 0U) {
        return tick;
    }
    return (uint32_t)(((uint64_t)tick * 1000ULL) / tick_freq);
}

static void task_lift_enter_sensor_fault(lift_fault_reason_t reason)
{
    task_lift_stop();
    s_lift_fault_reason = reason;
    lift.state = LIFT_STATE_FAULT;
    control_arbitration_release(&lift_arbitration);
}

static void task_lift_init_reference_sensors(void)
{
    lift_home_sensor_init(&lift_bottom_sensor);
    lift_home_sensor_init(&lift_middle_sensor);
    lift_home_sensor_init(&lift_top_sensor);
}

static void task_lift_update_reference_sensors(void)
{
    lift_home_sensor_update(&lift_bottom_sensor);
    lift_home_sensor_update(&lift_middle_sensor);
    lift_home_sensor_update(&lift_top_sensor);
}

static void task_lift_clear_reject(void)
{
    s_lift_reject_reason = LIFT_REJECT_NONE;
    s_lift_rejected_target_z = 0.0f;
}

static void task_lift_reject_move(lift_reject_reason_t reason, float z)
{
    s_lift_reject_reason = reason;
    s_lift_rejected_target_z = z;
}

static void task_lift_send_result(uint8_t seq,
                                  uint8_t cmd,
                                  uint8_t source,
                                  lift_result_t result,
                                  lift_reject_reason_t reject_reason,
                                  lift_fault_reason_t fault_reason,
                                  float requested_z,
                                  float accepted_z,
                                  uint32_t now_tick)
{
    ctrl_lift_result_payload_t payload = {0};

    payload.tick_ms = task_lift_tick_to_ms(now_tick);
    payload.request_seq = seq;
    payload.request_cmd = cmd;
    payload.request_source = source;
    payload.result = (uint8_t)result;
    payload.reject_reason = (uint8_t)reject_reason;
    payload.fault_reason = (uint8_t)fault_reason;
    payload.state_after = (uint8_t)lift.state;
    payload.requested_z = requested_z;
    payload.accepted_z = accepted_z;
    payload.current_z = lift.current_z;
    payload.z_min = lift.z_min;
    payload.z_max = lift.z_max;

    (void)telemetry_submit_result(CTRL_TARGET_LIFT, CTRL_LIFT_RPT_RESULT,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
}

static void task_lift_set_active_cmd(const lift_cmd_msg_t *msg, float accepted_z)
{
    if (msg == 0) {
        return;
    }

    s_lift_active_cmd.valid = 1U;
    s_lift_active_cmd.seq = msg->seq;
    s_lift_active_cmd.cmd = msg->type;
    s_lift_active_cmd.source = msg->source;
    s_lift_active_cmd.requested_z = msg->z;
    s_lift_active_cmd.accepted_z = accepted_z;
}

static void task_lift_clear_active_cmd(void)
{
    s_lift_active_cmd.valid = 0U;
    s_lift_active_cmd.seq = 0U;
    s_lift_active_cmd.cmd = LIFT_CMD_STOP;
    s_lift_active_cmd.source = CTRL_SRC_NONE;
    s_lift_active_cmd.requested_z = 0.0f;
    s_lift_active_cmd.accepted_z = 0.0f;
}

static void task_lift_send_active_result(lift_result_t result,
                                         lift_fault_reason_t fault_reason,
                                         uint32_t now_tick)
{
    if (s_lift_active_cmd.valid == 0U) {
        return;
    }

    task_lift_send_result(s_lift_active_cmd.seq,
                          s_lift_active_cmd.cmd,
                          s_lift_active_cmd.source,
                          result,
                          LIFT_REJECT_NONE,
                          fault_reason,
                          s_lift_active_cmd.requested_z,
                          s_lift_active_cmd.accepted_z,
                          now_tick);

    if (result != LIFT_RESULT_ACCEPTED) {
        task_lift_clear_active_cmd();
    }
}

static void task_lift_reject_msg(const lift_cmd_msg_t *msg,
                                 lift_reject_reason_t reason,
                                 uint32_t now_tick)
{
    float requested_z = 0.0f;

    if (msg == 0) {
        return;
    }

    if (msg->type == LIFT_CMD_MOVE_Z) {
        requested_z = msg->z;
        task_lift_reject_move(reason, requested_z);
    } else {
        task_lift_clear_reject();
    }

    task_lift_send_result(msg->seq,
                          msg->type,
                          msg->source,
                          LIFT_RESULT_REJECTED,
                          reason,
                          LIFT_FAULT_NONE,
                          requested_z,
                          lift.target_z,
                          now_tick);
}

static void task_lift_clear_ref_edge_lockout(void)
{
    lift_clear_reference_state(&lift);
}

static void task_lift_process_reference_event(lift_ref_sensor_e sensor,
                                              lift_ref_edge_e edge,
                                              uint32_t now_ms)
{
    lift_ref_result_e result;

    result = lift_process_reference_event(&lift, sensor, edge, now_ms);
    if (result == LIFT_REF_RESULT_MISMATCH) {
        task_lift_enter_sensor_fault(LIFT_FAULT_SENSOR_POSITION_MISMATCH);
    }
}

static void task_lift_process_reference_sensors(uint32_t now_tick)
{
    uint32_t now_ms = task_lift_tick_to_ms(now_tick);

    if (lift_middle_sensor.rising_edge != 0U) {
        task_lift_process_reference_event(LIFT_REF_SENSOR_MIDDLE,
                                          LIFT_REF_EDGE_RISING,
                                          now_ms);
    }
    if (lift_middle_sensor.falling_edge != 0U) {
        task_lift_process_reference_event(LIFT_REF_SENSOR_MIDDLE,
                                          LIFT_REF_EDGE_FALLING,
                                          now_ms);
    }
    if (lift_top_sensor.rising_edge != 0U) {
        task_lift_process_reference_event(LIFT_REF_SENSOR_TOP,
                                          LIFT_REF_EDGE_RISING,
                                          now_ms);
    }
    if (lift_top_sensor.falling_edge != 0U) {
        task_lift_process_reference_event(LIFT_REF_SENSOR_TOP,
                                          LIFT_REF_EDGE_FALLING,
                                          now_ms);
    }
}

static void task_lift_state_snapshot_update(uint32_t now_tick)
{
    lift_diag_code_t diag_code = LIFT_DIAG_OK;
    lift_diag_severity_t severity = LIFT_DIAG_SEV_INFO;
    uint32_t detail_u32 = 0UL;
    float detail_f32 = 0.0f;
    uint32_t tick_ms = task_lift_tick_to_ms(now_tick);
    float position_error = lift.target_z - lift.current_z;

    /* Summary is the first stop for Watch/uplink: it maps details to one cause. */
    switch (s_lift_fault_reason) {
    case LIFT_FAULT_HOME_SENSOR_NOT_INIT:
        diag_code = LIFT_DIAG_HOME_SENSOR_ERROR;
        severity = LIFT_DIAG_SEV_FATAL;
        detail_u32 = lift_is_home_sensor_initialized(&lift);
        break;
    case LIFT_FAULT_HOME_TIMEOUT:
        diag_code = LIFT_DIAG_HOME_TIMEOUT;
        severity = LIFT_DIAG_SEV_ERROR;
        detail_u32 = now_tick - lift_home_start_tick;
        detail_f32 = (float)lift_get_home_sensor_level(&lift);
        break;
    case LIFT_FAULT_MOTOR_BLOCKED:
        diag_code = LIFT_DIAG_MOTOR_BLOCKED;
        severity = LIFT_DIAG_SEV_FATAL;
        detail_u32 = lift_motor.blocked_cnt;
        detail_f32 = lift_motor.current_rpm;
        break;
    case LIFT_FAULT_CONTROL_TIMEOUT:
        diag_code = LIFT_DIAG_CONTROL_TIMEOUT;
        severity = LIFT_DIAG_SEV_ERROR;
        detail_f32 = position_error;
        break;
    case LIFT_FAULT_SENSOR_POSITION_MISMATCH:
        diag_code = LIFT_DIAG_POSITION_MISMATCH;
        severity = LIFT_DIAG_SEV_ERROR;
        detail_u32 = ((uint32_t)lift.last_ref_sensor << 8U) |
                     (uint32_t)lift.last_ref_edge;
        detail_f32 = lift.last_ref_error;
        break;
    case LIFT_FAULT_NONE:
    default:
        diag_code = ((lift.state == LIFT_STATE_HOMING) ||
                     (lift.state == LIFT_STATE_MOVING))
                        ? LIFT_DIAG_BUSY
                        : LIFT_DIAG_OK;
        break;
    }

    g_lift_state.summary.tick_ms = tick_ms;
    g_lift_state.summary.code = diag_code;
    g_lift_state.summary.severity = severity;
    g_lift_state.summary.fault_reason = s_lift_fault_reason;
    g_lift_state.summary.state = lift.state;
    g_lift_state.summary.home_state = lift.home_state;
    g_lift_state.summary.detail_u32 = detail_u32;
    g_lift_state.summary.detail_f32 = detail_f32;

    g_lift_state.runtime.tick_ms = tick_ms;
    g_lift_state.runtime.state = lift.state;
    g_lift_state.runtime.home_state = lift.home_state;
    g_lift_state.runtime.is_homed = lift.is_homed;
    g_lift_state.runtime.is_busy = ((lift.state == LIFT_STATE_HOMING) ||
                                    (lift.state == LIFT_STATE_MOVING))
                                       ? 1U
                                       : 0U;
    g_lift_state.runtime.has_fault = (lift.state == LIFT_STATE_FAULT) ? 1U : 0U;
    g_lift_state.runtime.current_z = lift.current_z;
    g_lift_state.runtime.target_z = lift.target_z;
    g_lift_state.runtime.current_v = lift.current_v;
    g_lift_state.runtime.target_v = lift.target_v;
    g_lift_state.runtime.position_error = position_error;
    g_lift_state.runtime.motor_total_encoder_count = lift_motor.total_encoder_count;

    g_lift_state.detail.fault_reason = s_lift_fault_reason;
    g_lift_state.detail.reject_reason = s_lift_reject_reason;
    g_lift_state.detail.rejected_target_z = s_lift_rejected_target_z;
    g_lift_state.detail.reject_z_min = lift.z_min;
    g_lift_state.detail.reject_z_max = lift.z_max;
    g_lift_state.detail.home_start_level = lift.home_start_level;
    g_lift_state.detail.bottom_sensor_initialized = lift_bottom_sensor.is_initialized;
    g_lift_state.detail.bottom_sensor_level = lift_bottom_sensor.level;
    g_lift_state.detail.bottom_sensor_last_level = lift_bottom_sensor.last_level;
    g_lift_state.detail.bottom_sensor_rising_edge = lift_bottom_sensor.rising_edge;
    g_lift_state.detail.bottom_sensor_falling_edge = lift_bottom_sensor.falling_edge;

    g_lift_state.detail.middle_sensor_initialized = lift_middle_sensor.is_initialized;
    g_lift_state.detail.middle_sensor_level = lift_middle_sensor.level;
    g_lift_state.detail.middle_sensor_last_level = lift_middle_sensor.last_level;
    g_lift_state.detail.middle_sensor_rising_edge = lift_middle_sensor.rising_edge;
    g_lift_state.detail.middle_sensor_falling_edge = lift_middle_sensor.falling_edge;

    g_lift_state.detail.top_sensor_initialized = lift_top_sensor.is_initialized;
    g_lift_state.detail.top_sensor_level = lift_top_sensor.level;
    g_lift_state.detail.top_sensor_last_level = lift_top_sensor.last_level;
    g_lift_state.detail.top_sensor_rising_edge = lift_top_sensor.rising_edge;
    g_lift_state.detail.top_sensor_falling_edge = lift_top_sensor.falling_edge;
    g_lift_state.detail.last_ref_sensor = (uint8_t)lift.last_ref_sensor;
    g_lift_state.detail.last_ref_edge = (uint8_t)lift.last_ref_edge;
    g_lift_state.detail.last_ref_z = lift.last_ref_z;
    g_lift_state.detail.last_ref_error = lift.last_ref_error;

    g_lift_state.detail.z_min = lift.z_min;
    g_lift_state.detail.z_max = lift.z_max;
    g_lift_state.detail.max_v = lift.max_v;
    g_lift_state.detail.reached_eps = lift.reached_eps;
    g_lift_state.detail.position_pid_out = lift.position_pid.out;

    g_lift_state.detail.motor_current_rpm = lift_motor.current_rpm;
    g_lift_state.detail.motor_target_rpm = lift_motor.target_rpm;
    g_lift_state.detail.motor_current_direction = lift_motor.current_direction;
    g_lift_state.detail.motor_target_direction = lift_motor.target_direction;
    g_lift_state.detail.motor_total_encoder_count = lift_motor.total_encoder_count;
    g_lift_state.detail.motor_initialized = lift_motor.is_initialized;
    g_lift_state.detail.motor_blocked = lift_motor.is_blocked;
    g_lift_state.detail.motor_blocked_cnt = lift_motor.blocked_cnt;
    g_lift_state.detail.motor_block_max_cnt = lift_motor.block_max_cnt;
    g_lift_state.detail.motor_pid_out = lift_motor.pid.out;

    g_lift_state.detail.home_start_tick = lift_home_start_tick;
    g_lift_state.detail.home_elapsed_tick = now_tick - lift_home_start_tick;
}

static void task_lift_send_status(void)
{
    ctrl_lift_status_payload_t payload = {0};
    uint32_t now_ms = g_lift_state.summary.tick_ms;

    if (s_lift_status_phase_ready == 0U) {
        if (s_lift_status_phase_start_ms == 0U) {
            s_lift_status_phase_start_ms = now_ms;
        }
        if ((uint32_t)(now_ms - s_lift_status_phase_start_ms) <
            LIFT_STATUS_PHASE_MS) {
            return;
        }
        s_lift_status_phase_ready = 1U;
        s_lift_status_last_ms = now_ms - LIFT_STATUS_PERIOD_MS;
    }

    if ((uint32_t)(now_ms - s_lift_status_last_ms) < LIFT_STATUS_PERIOD_MS) {
        return;
    }
    s_lift_status_last_ms = now_ms;

    payload.tick_ms = g_lift_state.summary.tick_ms;
    payload.state = (uint8_t)g_lift_state.runtime.state;
    payload.home_state = (uint8_t)g_lift_state.runtime.home_state;
    payload.is_homed = g_lift_state.runtime.is_homed;
    payload.is_busy = g_lift_state.runtime.is_busy;
    payload.has_fault = g_lift_state.runtime.has_fault;
    payload.fault_reason = (uint8_t)g_lift_state.summary.fault_reason;
    payload.motor_blocked = g_lift_state.detail.motor_blocked;
    payload.home_sensor_level = lift_get_home_sensor_level(&lift);
    payload.current_z = g_lift_state.runtime.current_z;
    payload.target_z = g_lift_state.runtime.target_z;
    payload.current_v = g_lift_state.runtime.current_v;
    payload.target_v = g_lift_state.runtime.target_v;
    payload.position_error = g_lift_state.runtime.position_error;
    payload.motor_total_encoder_count = g_lift_state.runtime.motor_total_encoder_count;

    (void)telemetry_submit_status(CTRL_TARGET_LIFT, CTRL_LIFT_RPT_STATUS,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
}

static void task_lift_stop(void)
{
    lift_stop(&lift);
}

static void task_lift_abort_motion(void)
{
    task_lift_stop();
    if (lift.state != LIFT_STATE_FAULT) {
        lift.state = LIFT_STATE_IDLE;
    }
    if (lift.is_homed == 0U) {
        lift.home_state = LIFT_HOME_STATE_IDLE;
    }
}

static void task_lift_clear_fault(void)
{
    task_lift_stop();
    task_lift_clear_reject();
    task_lift_clear_active_cmd();
    lift_motor.is_blocked = 0U;
    lift_motor.blocked_cnt = 0U;
    lift.is_homed = 0U;
    lift.state = LIFT_STATE_IDLE;
    lift.home_state = LIFT_HOME_STATE_IDLE;
    task_lift_clear_ref_edge_lockout();
    s_lift_fault_reason = LIFT_FAULT_NONE;
}

static uint8_t task_lift_can_move_z(void)
{
    if (lift.state == LIFT_STATE_FAULT) {
        return 0U;
    }

    return lift_is_homed(&lift);
}

static void task_lift_start_home(uint32_t now_tick)
{
    task_lift_clear_reject();
    task_lift_clear_ref_edge_lockout();
    lift_start_home(&lift);
    if (lift.state == LIFT_STATE_HOMING) {
        s_lift_fault_reason = LIFT_FAULT_NONE;
        lift_home_start_tick = now_tick;
    } else if (lift.state == LIFT_STATE_FAULT) {
        s_lift_fault_reason = LIFT_FAULT_HOME_SENSOR_NOT_INIT;
    }
}

static void task_lift_process_debug_move(uint32_t now_tick)
{
    if (g_lift_debug_move_trigger == 0U) {
        return;
    }

    g_lift_debug_move_trigger = 0U;
    if (task_lift_can_move_z() == 0U) {
        g_lift_debug_move_status = LIFT_DEBUG_MOVE_NOT_READY;
        task_lift_reject_move((lift.state == LIFT_STATE_FAULT) ? LIFT_REJECT_IN_FAULT : LIFT_REJECT_NOT_HOMED,
                              g_lift_debug_move_z);
        return;
    }
    if (lift_target_z_is_valid(&lift, g_lift_debug_move_z) == 0U) {
        g_lift_debug_move_status = LIFT_DEBUG_MOVE_OUT_OF_RANGE;
        task_lift_reject_move(LIFT_REJECT_TARGET_OUT_OF_RANGE,
                              g_lift_debug_move_z);
        s_lift_fault_reason = LIFT_FAULT_NONE;
        return;
    }
    if (control_arbitration_can_accept(&lift_arbitration, CTRL_SRC_MCU) == 0U) {
        g_lift_debug_move_status = LIFT_DEBUG_MOVE_BUSY;
        task_lift_reject_move(LIFT_REJECT_BUSY, g_lift_debug_move_z);
        return;
    }

    if (lift_set_target_z(&lift, g_lift_debug_move_z) == 0U) {
        g_lift_debug_move_status = LIFT_DEBUG_MOVE_OUT_OF_RANGE;
        task_lift_reject_move(LIFT_REJECT_TARGET_OUT_OF_RANGE,
                              g_lift_debug_move_z);
        s_lift_fault_reason = LIFT_FAULT_NONE;
        return;
    }
    g_lift_debug_move_target_z = lift.target_z;
    g_lift_debug_move_status = LIFT_DEBUG_MOVE_ACCEPTED;
    task_lift_clear_reject();
    s_lift_fault_reason = LIFT_FAULT_NONE;
    control_arbitration_accept(&lift_arbitration, CTRL_SRC_MCU, now_tick);
}

static void task_lift_handle_msg(const lift_cmd_msg_t *msg, uint32_t now_tick)
{
    uint8_t was_moving;

    if (msg == 0) {
        return;
    }

    switch (msg->type) {
    case LIFT_CMD_STOP:
        task_lift_abort_motion();
        task_lift_clear_reject();
        control_arbitration_release(&lift_arbitration);
        if (lift.state == LIFT_STATE_FAULT) {
            task_lift_send_active_result(LIFT_RESULT_FAILED,
                                         s_lift_fault_reason,
                                         now_tick);
        } else {
            task_lift_send_active_result(LIFT_RESULT_ABORTED,
                                         LIFT_FAULT_NONE,
                                         now_tick);
        }
        task_lift_send_result(msg->seq,
                              msg->type,
                              msg->source,
                              LIFT_RESULT_COMPLETED,
                              LIFT_REJECT_NONE,
                              LIFT_FAULT_NONE,
                              msg->z,
                              lift.target_z,
                              now_tick);
        break;

    case LIFT_CMD_HOME:
        if ((lift.state == LIFT_STATE_HOMING) ||
            (lift.state == LIFT_STATE_MOVING)) {
            task_lift_reject_msg(msg, LIFT_REJECT_BUSY, now_tick);
        } else if (control_arbitration_can_accept(&lift_arbitration,
                                                  (control_source_e)msg->source) != 0U) {
            task_lift_start_home(now_tick);
            if (lift.state == LIFT_STATE_HOMING) {
                task_lift_set_active_cmd(msg, lift.target_z);
                task_lift_send_active_result(LIFT_RESULT_ACCEPTED,
                                             LIFT_FAULT_NONE,
                                             now_tick);
                control_arbitration_accept(&lift_arbitration,
                                           (control_source_e)msg->source, now_tick);
            } else {
                task_lift_send_result(msg->seq,
                                      msg->type,
                                      msg->source,
                                      LIFT_RESULT_FAILED,
                                      LIFT_REJECT_NONE,
                                      s_lift_fault_reason,
                                      msg->z,
                                      lift.target_z,
                                      now_tick);
            }
        } else {
            task_lift_reject_msg(msg, LIFT_REJECT_BUSY, now_tick);
        }
        break;

    case LIFT_CMD_MOVE_Z:
        if (lift.state == LIFT_STATE_FAULT) {
            task_lift_reject_msg(msg, LIFT_REJECT_IN_FAULT, now_tick);
            break;
        }
        if (lift.state == LIFT_STATE_HOMING) {
            task_lift_reject_msg(msg, LIFT_REJECT_BUSY, now_tick);
            break;
        }
        if (task_lift_can_move_z() == 0U) {
            task_lift_reject_msg(msg, LIFT_REJECT_NOT_HOMED, now_tick);
            break;
        }
        if (lift_target_z_is_valid(&lift, msg->z) == 0U) {
            task_lift_reject_msg(msg, LIFT_REJECT_TARGET_OUT_OF_RANGE, now_tick);
            break;
        }
        if (control_arbitration_can_accept(&lift_arbitration,
                                           (control_source_e)msg->source) == 0U) {
            task_lift_reject_msg(msg, LIFT_REJECT_BUSY, now_tick);
            break;
        }

        was_moving = (lift.state == LIFT_STATE_MOVING) ? 1U : 0U;
        if (lift_set_target_z(&lift, msg->z) != 0U) {
            if (was_moving != 0U) {
                task_lift_send_active_result(LIFT_RESULT_SUPERSEDED,
                                             LIFT_FAULT_NONE,
                                             now_tick);
            }
            task_lift_set_active_cmd(msg, lift.target_z);
            task_lift_clear_reject();
            s_lift_fault_reason = LIFT_FAULT_NONE;
            task_lift_send_active_result(LIFT_RESULT_ACCEPTED,
                                         LIFT_FAULT_NONE,
                                         now_tick);
            control_arbitration_accept(&lift_arbitration,
                                       (control_source_e)msg->source, now_tick);
        } else {
            task_lift_reject_msg(msg, LIFT_REJECT_TARGET_OUT_OF_RANGE, now_tick);
        }
        break;

    case LIFT_CMD_CLEAR_FAULT:
        if (lift.state == LIFT_STATE_FAULT) {
            task_lift_send_active_result(LIFT_RESULT_FAILED,
                                         s_lift_fault_reason,
                                         now_tick);
        } else {
            task_lift_send_active_result(LIFT_RESULT_ABORTED,
                                         LIFT_FAULT_NONE,
                                         now_tick);
        }
        task_lift_clear_fault();
        control_arbitration_release(&lift_arbitration);
        task_lift_send_result(msg->seq,
                              msg->type,
                              msg->source,
                              LIFT_RESULT_COMPLETED,
                              LIFT_REJECT_NONE,
                              LIFT_FAULT_NONE,
                              msg->z,
                              lift.target_z,
                              now_tick);
        break;

    default:
        task_lift_reject_msg(msg, LIFT_REJECT_UNKNOWN_CMD, now_tick);
        break;
    }
}

static void task_lift_drain_queue(uint32_t now_tick)
{
    lift_cmd_msg_t msg;

    while (osMessageQueueGet(LiftCmdQueueHandle, &msg, 0U, 0U) == osOK) {
        task_lift_handle_msg(&msg, now_tick);
    }
}

static void task_lift_update_arbitration(uint32_t now_tick)
{
    if (control_arbitration_is_timeout(&lift_arbitration, now_tick,
                                       LIFT_CONTROL_TIMEOUT_MS) != 0U) {
        task_lift_stop();
        s_lift_fault_reason = LIFT_FAULT_CONTROL_TIMEOUT;
        lift.state = LIFT_STATE_FAULT;
        control_arbitration_release(&lift_arbitration);
    }

    if (lift_is_reached(&lift) != 0U) {
        control_arbitration_release(&lift_arbitration);
    }
}

static void task_lift_update_active_result(uint32_t now_tick)
{
    if (s_lift_active_cmd.valid == 0U) {
        return;
    }

    if (lift.state == LIFT_STATE_FAULT) {
        task_lift_send_active_result(LIFT_RESULT_FAILED,
                                     s_lift_fault_reason,
                                     now_tick);
    } else if (lift_is_reached(&lift) != 0U) {
        task_lift_send_active_result(LIFT_RESULT_COMPLETED,
                                     LIFT_FAULT_NONE,
                                     now_tick);
    }
}

static void task_lift_update_home_search_limits(void)
{
    if (lift.state != LIFT_STATE_HOMING) {
        return;
    }

    if ((lift.target_v > 0.0f) &&
        (lift_top_sensor.level == LIFT_HOME_SENSOR_BLOCKED_LEVEL)) {
        lift.home_start_level = lift_get_home_sensor_level(&lift);
        lift.home_state = LIFT_HOME_STATE_WAIT_FALLING_EDGE;
        lift_set_velocity(&lift, -LIFT_HOME_V_MPS);
    } else if ((lift.target_v < 0.0f) &&
               (lift_bottom_sensor.level == LIFT_HOME_SENSOR_BLOCKED_LEVEL)) {
        lift.home_start_level = lift_get_home_sensor_level(&lift);
        lift.home_state = LIFT_HOME_STATE_WAIT_FALLING_EDGE;
        lift_set_velocity(&lift, LIFT_HOME_V_MPS);
    }
}

static void task_lift_update_home_timeout(uint32_t now_tick)
{
    if ((lift.state == LIFT_STATE_HOMING) &&
        ((uint32_t)(now_tick - lift_home_start_tick) > LIFT_HOME_TIMEOUT_MS)) {
        task_lift_stop();
        s_lift_fault_reason = LIFT_FAULT_HOME_TIMEOUT;
        lift.home_state = LIFT_HOME_STATE_FAULT;
        lift.state = LIFT_STATE_FAULT;
        control_arbitration_release(&lift_arbitration);
    }
}

void StartLiftTask(void *argument)
{
    uint32_t now_tick;

    (void)argument;

    control_arbitration_init(&lift_arbitration);
    lift_init(&lift);
    task_lift_init_reference_sensors();
    task_lift_start_home(osKernelGetTickCount());

    for (;;) {
        now_tick = osKernelGetTickCount();

        task_lift_drain_queue(now_tick);
        task_lift_process_debug_move(now_tick);
        task_lift_update_reference_sensors();
        task_lift_update_home_search_limits();

        if (lift_motor.is_blocked != 0U) {
            task_lift_stop();
            s_lift_fault_reason = LIFT_FAULT_MOTOR_BLOCKED;
            lift.state = LIFT_STATE_FAULT;
            control_arbitration_release(&lift_arbitration);
        } else {
            lift_update(&lift, LIFT_TASK_DT);
        }

        task_lift_process_reference_sensors(now_tick);
        task_lift_update_home_timeout(now_tick);
        task_lift_update_arbitration(now_tick);
        task_lift_update_active_result(now_tick);
        task_lift_state_snapshot_update(now_tick);
        task_lift_send_status();

        osDelayUntil(now_tick + LIFT_TASK_PERIOD_MS);
    }
}
