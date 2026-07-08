#ifndef _MDL_SO101_ARM_H_
#define _MDL_SO101_ARM_H_

#include <stdint.h>

#include "dev_sts_servo.h"
#include "dev_uart.h"
#include "so101_algorithm.h"

typedef enum {
    SO101_ARM_OK = 0,
    SO101_ARM_ERR_NULL = -1,
    SO101_ARM_ERR_PARAM = -2,
    SO101_ARM_ERR_IK = -3,
    SO101_ARM_ERR_FAULT = -4,
} so101_arm_status_t;

typedef enum {
    SO101_ARM_MODE_JOINT = 0,
    SO101_ARM_MODE_CARTESIAN,
    SO101_ARM_MODE_POSE,
} so101_arm_mode_t;

typedef enum {
    SO101_ARM_STATE_IDLE = 0,
    SO101_ARM_STATE_MOVING,
    SO101_ARM_STATE_REACHED,
    SO101_ARM_STATE_FAULT,
} so101_arm_state_t;

typedef struct {
    float x;
    float y;
    float z;
} so101_arm_position_t;

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
} so101_arm_pose_t;

/* Non-moving target validation used before accepting a command. */
typedef enum {
    SO101_ARM_REACH_OK = 0,
    SO101_ARM_REACH_ERR_NULL,
    SO101_ARM_REACH_ERR_XYZ_RANGE,
    SO101_ARM_REACH_ERR_Z_LOW,
    SO101_ARM_REACH_ERR_Z_HIGH,
    SO101_ARM_REACH_ERR_IK,
    SO101_ARM_REACH_ERR_LIMIT_MARGIN,
} so101_arm_reach_status_t;

typedef struct {
    so101_arm_reach_status_t status;
    float joint_rad[SO101_ACTIVE_JOINT_COUNT];
    float position_error_m;
    float pitch_error_rad;
    float limit_margin_rad;
    float side_error_m;
    uint16_t ik_iterations;
} so101_arm_reach_result_t;

typedef struct so101_arm_joint_class {
    void *ctx;
    void (*init)(void *ctx);
    void (*set_angle_rad)(void *ctx, float angle_rad);
    float (*get_angle_rad)(void *ctx);
    void (*enable_torque)(void *ctx, uint8_t enable);
    void (*update)(void *ctx);
    uint32_t (*get_faults)(void *ctx);
} so101_arm_joint_t;

typedef struct so101_arm_joint_group_class {
    void *ctx;
    void (*init)(void *ctx);
    void (*update)(void *ctx);
    void (*enable_torque)(void *ctx, uint8_t enable);
} so101_arm_joint_group_t;

typedef struct so101_arm_gripper_class {
    void *ctx;
    void (*init)(void *ctx);
    void (*set_angle_rad)(void *ctx, float angle_rad);
    float (*get_angle_rad)(void *ctx);
    void (*enable_torque)(void *ctx, uint8_t enable);
    uint32_t (*get_faults)(void *ctx);
} so101_arm_gripper_t;

typedef struct {
    sts_servo_bus_t *bus;
    sts_servo_t *servos;
    uint8_t servo_count;
    const uart_driver_t *uart;
    uint32_t timeout_ms;
} so101_arm_servo_group_t;

typedef struct so101_arm_class {
    so101_arm_mode_t mode;
    so101_arm_state_t state;
    uint8_t is_initialized;
    uint8_t target_dirty;

    so101_arm_position_t current_position;
    so101_arm_position_t target_position;
    so101_arm_pose_t target_pose;

    float current_joint_rad[SO101_ACTIVE_JOINT_COUNT];
    float target_joint_rad[SO101_ACTIVE_JOINT_COUNT];
    float command_joint_rad[SO101_ACTIVE_JOINT_COUNT];

    float max_joint_step_rad;
    float reached_angle_eps_rad;
    float reached_pos_eps_m;

    uint32_t fault_flags;
    uint32_t fatal_fault_flags;
    uint32_t nonfatal_joint_fault_mask;
    robot_status_t last_robot_status;
    robot_ik_options_t ik_options;
    robot_ik_info_t ik_info;

    so101_arm_joint_t joint[SO101_ACTIVE_JOINT_COUNT];
    so101_arm_joint_group_t joint_group;
    so101_arm_gripper_t gripper;
} so101_arm_t;

void so101_arm_init(so101_arm_t *arm);
void so101_arm_servo_group_init(void *ctx);
void so101_arm_servo_group_enable_torque(void *ctx, uint8_t enable);
/* Target setters only accept a request. Execution, IK status, and reach state are reported by update/status APIs. */
so101_arm_status_t so101_arm_set_joint_angles(so101_arm_t *arm,
                                              const float joint_rad[SO101_ACTIVE_JOINT_COUNT]);
so101_arm_status_t so101_arm_set_position(so101_arm_t *arm,
                                          const so101_arm_position_t *position);
so101_arm_status_t so101_arm_set_pose(so101_arm_t *arm,
                                      const so101_arm_pose_t *pose);
void so101_arm_enable_torque(so101_arm_t *arm, uint8_t enable);
void so101_arm_set_gripper_angle(so101_arm_t *arm, float gripper_rad);
float so101_arm_get_gripper_angle(const so101_arm_t *arm);
so101_arm_status_t so101_arm_update(so101_arm_t *arm, float dt);
uint8_t so101_arm_is_reached(const so101_arm_t *arm);
uint32_t so101_arm_get_faults(const so101_arm_t *arm);
so101_arm_reach_status_t so101_arm_check_reachable(so101_arm_t *arm,
                                                   const so101_arm_position_t *target,
                                                   so101_arm_reach_result_t *result);

#endif /* _MDL_SO101_ARM_H_ */
