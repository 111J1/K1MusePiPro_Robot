#include "task_chassis.h"

#include "dev_brush_motor.h"
#include "drv_brush_motor.h"
#include "mdl_chassis.h"
#include "mdl_control_arbitration.h"
#include "task_telemetry.h"

#define CHASSIS_TASK_PERIOD_MS (10U)
#define CHASSIS_STATUS_PERIOD_MS (50U)
#define CHASSIS_STATUS_PHASE_MS (0U)
#define CHASSIS_CONTROL_TIMEOUT_MS (300U)
#define CHASSIS_TASK_DT (0.01f)

brush_motor_t motor[MOTOR_COUNT] = {
    [MOTOR_LF] = {
        .pid = {
            .kp = 0.5f,
            .ki = 0.1f,
            .kd = 0.f,
            .out_min = -100.f,
            .out_max = 100.f,
        },
        .max_rpm = 330.f,
        .block_max_cnt = 50U,
        .one_round_encoder_count = 1320U,
        .driver = &LF_motor_driver,
    },
    [MOTOR_RF] = {
        .pid = {
            .kp = 0.5f,
            .ki = 0.1f,
            .kd = 0.f,
            .out_min = -100.f,
            .out_max = 100.f,
        },
        .max_rpm = 330.f,
        .block_max_cnt = 50U,
        .one_round_encoder_count = 1320U,
        .driver = &RF_motor_driver,
    },
    [MOTOR_RB] = {
        .pid = {
            .kp = 0.5f,
            .ki = 0.1f,
            .kd = 0.f,
            .out_min = -100.f,
            .out_max = 100.f,
        },
        .max_rpm = 330.f,
        .block_max_cnt = 50U,
        .one_round_encoder_count = 1320U,
        .driver = &RB_motor_driver,
    },
    [MOTOR_LB] = {
        .pid = {
            .kp = 0.5f,
            .ki = 0.1f,
            .kd = 0.f,
            .out_min = -100.f,
            .out_max = 100.f,
        },
        .max_rpm = 330.f,
        .block_max_cnt = 50U,
        .one_round_encoder_count = 1320U,
        .driver = &LB_motor_driver,
    },
};

chassis_t chassis = {
    .move_CS = CHASSIS_MOVE_MODE_LCS,
    .motor[MOTOR_LF] = {
        .ctx = &motor[MOTOR_LF],
        .init = brush_motor_init,
        .set_rpm = brush_motor_set_rpm,
        .get_rpm = brush_motor_get_rpm,
        .update = brush_motor_update,
    },
    .motor[MOTOR_RF] = {
        .ctx = &motor[MOTOR_RF],
        .init = brush_motor_init,
        .set_rpm = brush_motor_set_rpm,
        .get_rpm = brush_motor_get_rpm,
        .update = brush_motor_update,
    },
    .motor[MOTOR_RB] = {
        .ctx = &motor[MOTOR_RB],
        .init = brush_motor_init,
        .set_rpm = brush_motor_set_rpm,
        .get_rpm = brush_motor_get_rpm,
        .update = brush_motor_update,
    },
    .motor[MOTOR_LB] = {
        .ctx = &motor[MOTOR_LB],
        .init = brush_motor_init,
        .set_rpm = brush_motor_set_rpm,
        .get_rpm = brush_motor_get_rpm,
        .update = brush_motor_update,
    },
};

static module_control_arbitration_t chassis_arbitration;
static uint8_t chassis_control_timeout = 0U;
static uint32_t s_chassis_status_phase_start_ms = 0U;
static uint32_t s_chassis_status_last_ms = 0U;
static uint8_t s_chassis_status_phase_ready = 0U;

static void task_chassis_stop(void)
{
    /* STOP always uses local coordinates because velocity is zero. */
    chassis_set_movement(&chassis, CHASSIS_MOVE_MODE_LCS, 0.0f, 0.0f, 0.0f);
}

static chassis_move_CS_e task_chassis_convert_move_cs(uint8_t move_cs)
{
    return (move_cs == CTRL_CHS_MOVE_WCS) ? CHASSIS_MOVE_MODE_WCS : CHASSIS_MOVE_MODE_LCS;
}

static uint8_t task_chassis_get_move_cs(void)
{
    return (chassis.move_CS == CHASSIS_MOVE_MODE_WCS) ? CTRL_CHS_MOVE_WCS : CTRL_CHS_MOVE_LCS;
}

static uint8_t task_chassis_get_motor_block_flags(void)
{
    uint8_t flags = 0U;

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
        if (motor[i].is_blocked != 0U) {
            flags |= (uint8_t)(1U << i);
        }
    }

    return flags;
}

static uint8_t task_chassis_get_state(uint8_t motor_block_flags)
{
    if (motor_block_flags != 0U) {
        return CTRL_CHS_STATE_FAULT;
    }
    if (chassis_control_timeout != 0U) {
        return CTRL_CHS_STATE_TIMEOUT;
    }
    if ((chassis.current_V > 0.0f) || (chassis.current_omega != 0.0f)) {
        return CTRL_CHS_STATE_MOVING;
    }

    return CTRL_CHS_STATE_IDLE;
}

static void task_chassis_send_status(uint32_t now_tick)
{
    ctrl_chassis_status_payload_t payload = {0};
    uint8_t motor_block_flags = task_chassis_get_motor_block_flags();

    if (s_chassis_status_phase_ready == 0U) {
        if (s_chassis_status_phase_start_ms == 0U) {
            s_chassis_status_phase_start_ms = now_tick;
        }
        if ((uint32_t)(now_tick - s_chassis_status_phase_start_ms) <
            CHASSIS_STATUS_PHASE_MS) {
            return;
        }
        s_chassis_status_phase_ready = 1U;
        s_chassis_status_last_ms = now_tick - CHASSIS_STATUS_PERIOD_MS;
    }

    if ((uint32_t)(now_tick - s_chassis_status_last_ms) <
        CHASSIS_STATUS_PERIOD_MS) {
        return;
    }
    s_chassis_status_last_ms = now_tick;

    payload.tick_ms = now_tick;
    payload.state = task_chassis_get_state(motor_block_flags);
    payload.move_cs = task_chassis_get_move_cs();
    payload.motor_block_flags = motor_block_flags;
    payload.WCS_vx = chassis.current_WCS_Vx;
    payload.WCS_vy = chassis.current_WCS_Vy;
    payload.omega = chassis.current_omega;
    payload.WCS_x = chassis.current_WCS_X;
    payload.WCS_y = chassis.current_WCS_Y;
    payload.WCS_direction = chassis.current_WCS_direction;

    (void)telemetry_submit_status(CTRL_TARGET_CHASSIS, CTRL_CHS_RPT_STATUS,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
    chassis_control_timeout = 0U;
}

static void task_chassis_handle_msg(const chassis_cmd_msg_t *msg, uint32_t now_tick)
{
    if (msg == 0) {
        return;
    }

    switch (msg->type) {
    case CHASSIS_CMD_STOP:
        /* STOP is always accepted and clears module ownership. */
        task_chassis_stop();
        control_arbitration_release(&chassis_arbitration);
        break;

    case CHASSIS_CMD_MOV:
        /* Motion commands are exclusive to the current active source. */
        if (control_arbitration_can_accept(&chassis_arbitration,
                                           (control_source_e)msg->source) != 0U) {
            chassis_set_movement(&chassis, task_chassis_convert_move_cs(msg->move_cs),
                                 msg->direction, msg->v, msg->omega);
            control_arbitration_accept(&chassis_arbitration,
                                       (control_source_e)msg->source, now_tick);
        }
        break;

    case CHASSIS_CMD_ODOM:
        /* Odometry reset is allowed from any source and does not affect ownership. */
        chassis_reset_WCS_and_odometry(&chassis, msg->direction, msg->v, msg->omega);
        break;

    default:
        break;
    }
}

static void task_chassis_drain_queue(uint32_t now_tick)
{
    chassis_cmd_msg_t msg;

    /* Drain all queued commands so stale remote-control packets do not lag behind. */
    while (osMessageQueueGet(ChassisCmdQueueHandle, &msg, 0U, 0U) == osOK) {
        task_chassis_handle_msg(&msg, now_tick);
    }
}

void StartChassisTask(void *argument)
{
    /* USER CODE BEGIN StartChassisTask */
    (void)argument;

    control_arbitration_init(&chassis_arbitration);
    chassis_init(&chassis);
    task_chassis_stop();

    for (;;) {
        uint32_t now_tick = osKernelGetTickCount();

        task_chassis_drain_queue(now_tick);

        if (control_arbitration_is_timeout(&chassis_arbitration, now_tick,
                                           CHASSIS_CONTROL_TIMEOUT_MS) != 0U) {
            /* Fail safe: stop if the active controller stops refreshing MOV. */
            task_chassis_stop();
            control_arbitration_release(&chassis_arbitration);
            chassis_control_timeout = 1U;
        }

        chassis_update(&chassis, CHASSIS_TASK_DT);
        task_chassis_send_status(now_tick);

        osDelayUntil(now_tick + CHASSIS_TASK_PERIOD_MS);
    }
    /* USER CODE END StartChassisTask */
}
