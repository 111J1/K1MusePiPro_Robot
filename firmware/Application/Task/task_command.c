#include "task_command.h"

#include "drv_uart.h"
#include "mdl_control_protocol.h"
#include "task_arm.h"
#include "task_chassis.h"
#include "task_demo.h"
#include "task_lift.h"
#include "task_telemetry.h"
#include <string.h>

#define COMMAND_TASK_PERIOD_MS (10U)
#define COMMAND_RX_BUF_SIZE (128U)

static uint8_t command_rx_buf[COMMAND_RX_BUF_SIZE];
control_protocol_t bt_protocol;
control_protocol_t host_protocol;
volatile command_uart_state_t g_command_uart_state;

static uint8_t command_source_matches(const ctrl_frame_t *frame,
                                      control_source_e physical_source)
{
    /* Reject frames whose declared source does not match the UART they came from. */
    return (frame->src == (uint8_t)physical_source) ? 1U : 0U;
}

static uint8_t command_chassis_move_cs_is_valid(uint8_t move_cs)
{
    return ((move_cs == CTRL_CHS_MOVE_LCS) || (move_cs == CTRL_CHS_MOVE_WCS)) ? 1U : 0U;
}

static void command_put_chassis_msg(const chassis_cmd_msg_t *msg)
{
    if (msg != 0) {
        (void)osMessageQueuePut(ChassisCmdQueueHandle, msg, 0U, 0U);
    }
}

static void command_put_arm_msg(const arm_cmd_msg_t *msg)
{
    if (msg != 0) {
        (void)osMessageQueuePut(ArmCmdQueueHandle, msg, 0U, 0U);
    }
}

static void command_put_lift_msg(const lift_cmd_msg_t *msg)
{
    if (msg != 0) {
        (void)osMessageQueuePut(LiftCmdQueueHandle, msg, 0U, 0U);
    }
}

static uint32_t command_tick_to_ms(uint32_t tick)
{
    uint32_t freq = osKernelGetTickFreq();

    if (freq == 0U) {
        return tick;
    }
    return (uint32_t)(((uint64_t)tick * 1000ULL) / (uint64_t)freq);
}

static uint8_t command_demo_type_to_sys_cmd(uint8_t type)
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

static void command_send_demo_result(const demo_cmd_msg_t *msg,
                                     ctrl_result_e result,
                                     ctrl_demo_reject_reason_e reject_reason,
                                     demo_fault_e fault_reason)
{
    ctrl_demo_result_payload_t payload = {0};

    if (msg == 0) {
        return;
    }

    payload.tick_ms = command_tick_to_ms(osKernelGetTickCount());
    payload.request_seq = msg->seq;
    payload.request_cmd = command_demo_type_to_sys_cmd(msg->type);
    payload.request_source = msg->source;
    payload.result = (uint8_t)result;
    payload.reject_reason = (uint8_t)reject_reason;
    payload.fault_reason = (uint8_t)fault_reason;
    payload.state_after = (uint8_t)g_demo_state.state;
    payload.demo_id = msg->demo_id;
    payload.src_layer = msg->src_layer;
    payload.dst_layer = msg->dst_layer;
    payload.variant = msg->variant;
    payload.step_index = g_demo_state.step_index;
    payload.step_count = g_demo_state.step_count;

    (void)telemetry_submit_result(CTRL_TARGET_SYSTEM, CTRL_SYS_RPT_DEMO_RESULT,
                                  (const uint8_t *)&payload, (uint8_t)sizeof(payload));
}

static uint8_t command_put_demo_msg(const demo_cmd_msg_t *msg)
{
    if (msg != 0) {
        return (osMessageQueuePut(DemoCmdQueueHandle, msg, 0U, 0U) == osOK) ? 1U : 0U;
    }
    return 0U;
}

static void command_dispatch_system(const ctrl_frame_t *frame)
{
    demo_cmd_msg_t msg = {0};

    if ((frame == 0) || (frame->target != CTRL_TARGET_SYSTEM)) {
        return;
    }

    msg.source = frame->src;
    msg.seq = frame->seq;
    msg.tick = osKernelGetTickCount();

    switch (frame->cmd) {
    case CTRL_SYS_CMD_DEMO_STOP:
        if (frame->len == 0U) {
            msg.type = DEMO_CMD_STOP;
            if (command_put_demo_msg(&msg) != 0U) {
                command_send_demo_result(&msg, CTRL_RESULT_ACCEPTED,
                                         CTRL_DEMO_REJECT_NONE,
                                         DEMO_FAULT_NONE);
            } else {
                command_send_demo_result(&msg, CTRL_RESULT_REJECTED,
                                         CTRL_DEMO_REJECT_QUEUE_FULL,
                                         DEMO_FAULT_QUEUE_FULL);
            }
        }
        break;

    case CTRL_SYS_CMD_DEMO_RUN:
        if (frame->len == sizeof(ctrl_demo_run_payload_t)) {
            ctrl_demo_run_payload_t payload;

            memcpy(&payload, frame->payload, sizeof(payload));
            msg.type = DEMO_CMD_RUN;
            msg.demo_id = payload.demo_id;
            msg.src_layer = payload.src_layer;
            msg.dst_layer = payload.dst_layer;
            msg.variant = payload.variant;
            if (command_put_demo_msg(&msg) != 0U) {
                command_send_demo_result(&msg, CTRL_RESULT_ACCEPTED,
                                         CTRL_DEMO_REJECT_NONE,
                                         DEMO_FAULT_NONE);
            } else {
                command_send_demo_result(&msg, CTRL_RESULT_REJECTED,
                                         CTRL_DEMO_REJECT_QUEUE_FULL,
                                         DEMO_FAULT_QUEUE_FULL);
            }
        } else {
            msg.type = DEMO_CMD_RUN;
            command_send_demo_result(&msg, CTRL_RESULT_REJECTED,
                                     CTRL_DEMO_REJECT_BAD_LENGTH,
                                     DEMO_FAULT_NONE);
        }
        break;

    case CTRL_SYS_CMD_DEMO_HOME:
        if (frame->len == 0U) {
            msg.type = DEMO_CMD_HOME;
            if (command_put_demo_msg(&msg) != 0U) {
                command_send_demo_result(&msg, CTRL_RESULT_ACCEPTED,
                                         CTRL_DEMO_REJECT_NONE,
                                         DEMO_FAULT_NONE);
            } else {
                command_send_demo_result(&msg, CTRL_RESULT_REJECTED,
                                         CTRL_DEMO_REJECT_QUEUE_FULL,
                                         DEMO_FAULT_QUEUE_FULL);
            }
        }
        break;

    default:
        break;
    }
}

static void command_dispatch_chassis(const ctrl_frame_t *frame)
{
    chassis_cmd_msg_t msg = {0};

    if ((frame == 0) || (frame->target != CTRL_TARGET_CHASSIS)) {
        return;
    }

    msg.source = frame->src;
    msg.tick = osKernelGetTickCount();

    switch (frame->cmd) {
    case CTRL_CHS_CMD_STOP:
        /* STOP has no payload and intentionally maps to zero-initialized msg. */
        if (frame->len == 0U) {
            msg.type = CHASSIS_CMD_STOP;
            command_put_chassis_msg(&msg);
        }
        break;

    case CTRL_CHS_CMD_MOV:
        if (frame->len == sizeof(ctrl_chassis_mov_payload_t)) {
            ctrl_chassis_mov_payload_t payload;

            /* Payload is packed binary data defined by the protocol document. */
            memcpy(&payload, frame->payload, sizeof(payload));
            if (command_chassis_move_cs_is_valid(payload.move_cs) == 0U) {
                return;
            }

            msg.type = CHASSIS_CMD_MOV;
            msg.move_cs = payload.move_cs;
            msg.direction = payload.direction;
            msg.v = payload.v;
            msg.omega = payload.omega;
            command_put_chassis_msg(&msg);
        }
        break;

    case CTRL_CHS_CMD_ODOM:
        if (frame->len == sizeof(ctrl_chassis_odom_payload_t)) {
            ctrl_chassis_odom_payload_t payload;

            /* Reuse chassis queue float fields as direction, x, y. */
            memcpy(&payload, frame->payload, sizeof(payload));
            msg.type = CHASSIS_CMD_ODOM;
            msg.direction = payload.direction;
            msg.v = payload.x;
            msg.omega = payload.y;
            command_put_chassis_msg(&msg);
        }
        break;

    default:
        break;
    }
}

static void command_dispatch_arm(const ctrl_frame_t *frame)
{
    arm_cmd_msg_t msg = {0};

    if ((frame == 0) || (frame->target != CTRL_TARGET_ARM)) {
        return;
    }

    msg.source = frame->src;
    msg.seq = frame->seq;
    msg.tick = osKernelGetTickCount();

    switch (frame->cmd) {
    case CTRL_ARM_CMD_STOP:
        if (frame->len == 0U) {
            msg.type = ARM_CMD_STOP;
            command_put_arm_msg(&msg);
        }
        break;

    case CTRL_ARM_CMD_HOME:
        if (frame->len == 0U) {
            msg.type = ARM_CMD_HOME;
            command_put_arm_msg(&msg);
        }
        break;

    case CTRL_ARM_CMD_MOVE_XYZ:
        if (frame->len == sizeof(ctrl_arm_move_xyz_payload_t)) {
            ctrl_arm_move_xyz_payload_t payload;

            memcpy(&payload, frame->payload, sizeof(payload));
            msg.type = ARM_CMD_MOVE_XYZ;
            msg.x = payload.x;
            msg.y = payload.y;
            msg.z = payload.z;
            command_put_arm_msg(&msg);
        }
        break;

    case CTRL_ARM_CMD_MOVE_POSE:
        if (frame->len == sizeof(ctrl_arm_move_pose_payload_t)) {
            ctrl_arm_move_pose_payload_t payload;

            memcpy(&payload, frame->payload, sizeof(payload));
            msg.type = ARM_CMD_MOVE_POSE;
            msg.x = payload.x;
            msg.y = payload.y;
            msg.z = payload.z;
            msg.roll = payload.roll;
            msg.pitch = payload.pitch;
            command_put_arm_msg(&msg);
        }
        break;

    case CTRL_ARM_CMD_GRIPPER:
        if (frame->len == sizeof(ctrl_arm_gripper_payload_t)) {
            ctrl_arm_gripper_payload_t payload;

            memcpy(&payload, frame->payload, sizeof(payload));
            msg.type = ARM_CMD_GRIPPER;
            msg.gripper_rad = payload.gripper_rad;
            command_put_arm_msg(&msg);
        }
        break;

    case CTRL_ARM_CMD_CLEAR_FAULT:
        if (frame->len == 0U) {
            msg.type = ARM_CMD_CLEAR_FAULT;
            command_put_arm_msg(&msg);
        }
        break;

    case CTRL_ARM_CMD_DISABLE_TORQUE:
        if (frame->len == 0U) {
            msg.type = ARM_CMD_DISABLE_TORQUE;
            command_put_arm_msg(&msg);
        }
        break;

    default:
        break;
    }
}

static void command_dispatch_lift(const ctrl_frame_t *frame)
{
    lift_cmd_msg_t msg = {0};

    if ((frame == 0) || (frame->target != CTRL_TARGET_LIFT)) {
        return;
    }

    msg.source = frame->src;
    msg.seq = frame->seq;
    msg.tick = osKernelGetTickCount();

    switch (frame->cmd) {
    case CTRL_LIFT_CMD_STOP:
        if (frame->len == 0U) {
            msg.type = LIFT_CMD_STOP;
            command_put_lift_msg(&msg);
        }
        break;

    case CTRL_LIFT_CMD_HOME:
        if (frame->len == 0U) {
            msg.type = LIFT_CMD_HOME;
            command_put_lift_msg(&msg);
        }
        break;

    case CTRL_LIFT_CMD_MOVE_Z:
        if (frame->len == sizeof(ctrl_lift_z_payload_t)) {
            ctrl_lift_z_payload_t payload;

            memcpy(&payload, frame->payload, sizeof(payload));
            msg.type = LIFT_CMD_MOVE_Z;
            msg.z = payload.z;
            command_put_lift_msg(&msg);
        }
        break;

    case CTRL_LIFT_CMD_CLEAR_FAULT:
        if (frame->len == 0U) {
            msg.type = LIFT_CMD_CLEAR_FAULT;
            command_put_lift_msg(&msg);
        }
        break;

    default:
        break;
    }
}

static void command_dispatch_frame(const ctrl_frame_t *frame,
                                   control_source_e physical_source)
{
    if ((frame == 0) || (command_source_matches(frame, physical_source) == 0U)) {
        return;
    }

    if (physical_source == CTRL_SRC_HOST) {
        g_command_uart_state.host_last_valid_tick = osKernelGetTickCount();
    } else if (physical_source == CTRL_SRC_BT) {
        g_command_uart_state.bt_last_valid_tick = osKernelGetTickCount();
    }

    switch (frame->target) {
    case CTRL_TARGET_SYSTEM:
        command_dispatch_system(frame);
        break;

    case CTRL_TARGET_CHASSIS:
        command_dispatch_chassis(frame);
        break;

    case CTRL_TARGET_ARM:
        command_dispatch_arm(frame);
        break;

    case CTRL_TARGET_LIFT:
        command_dispatch_lift(frame);
        break;

    default:
        break;
    }
}

static void command_poll_uart(const uart_driver_t *uart,
                              control_protocol_t *protocol,
                              control_source_e physical_source)
{
    uint16_t len;
    ctrl_frame_t frame;

    if ((uart == 0) || (protocol == 0)) {
        return;
    }

    while (uart->available(uart->ctx) > 0U) {
        len = uart->read(uart->ctx, command_rx_buf, sizeof(command_rx_buf));
        if (len == 0U) {
            break;
        }
        /* Feed bytes one by one because UART reads may contain partial/multiple frames. */
        for (uint16_t i = 0U; i < len; ++i) {
            if (control_protocol_input_byte(protocol, command_rx_buf[i], &frame) != 0U) {
                command_dispatch_frame(&frame, physical_source);
            }
        }
    }
}

void StartCommandTask(void *argument)
{
    /* USER CODE BEGIN StartCommandTask */
    (void)argument;

    control_protocol_init(&bt_protocol);
    control_protocol_init(&host_protocol);

    /* USART3 is HC-05/BT, USART2 is reserved for the host computer. */
    (void)uart2_driver.init(uart2_driver.ctx);
    (void)uart3_driver.init(uart3_driver.ctx);

    for (;;) {
        uint32_t now_tick = osKernelGetTickCount();

        command_poll_uart(&uart3_driver, &bt_protocol, CTRL_SRC_BT);
        command_poll_uart(&uart2_driver, &host_protocol, CTRL_SRC_HOST);

        osDelayUntil(now_tick + COMMAND_TASK_PERIOD_MS);
    }
    /* USER CODE END StartCommandTask */
}
