#include "mdl_so101_arm.h"
#include "mdl_so101_arm_config.h"

#include <math.h>
#include <string.h>

static const float so101_arm_joint_min[SO101_ACTIVE_JOINT_COUNT] = {
    SO101_ARM_JOINT_1_MIN_RAD,
    SO101_ARM_JOINT_2_MIN_RAD,
    SO101_ARM_JOINT_3_MIN_RAD,
    SO101_ARM_JOINT_4_MIN_RAD,
    SO101_ARM_JOINT_5_MIN_RAD,
};

static const float so101_arm_joint_max[SO101_ACTIVE_JOINT_COUNT] = {
    SO101_ARM_JOINT_1_MAX_RAD,
    SO101_ARM_JOINT_2_MAX_RAD,
    SO101_ARM_JOINT_3_MAX_RAD,
    SO101_ARM_JOINT_4_MAX_RAD,
    SO101_ARM_JOINT_5_MAX_RAD,
};

static float so101_arm_clampf(float value, float min, float max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

static float so101_arm_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* Fixed fallback seed used only for target validation. */
static const float so101_arm_home_seed[SO101_ACTIVE_JOINT_COUNT] = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
};

static float so101_arm_joint_limit_margin(const float joint_rad[SO101_ACTIVE_JOINT_COUNT])
{
    float min_margin = 1000.0f;

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        float lower_margin = joint_rad[i] - so101_arm_joint_min[i];
        float upper_margin = so101_arm_joint_max[i] - joint_rad[i];
        float margin = (lower_margin < upper_margin) ? lower_margin : upper_margin;

        if (margin < min_margin) {
            min_margin = margin;
        }
    }

    return min_margin;
}

static so101_arm_reach_status_t so101_arm_check_workspace_bounds(const so101_arm_position_t *target)
{
    float r;

    if (target == 0) {
        return SO101_ARM_REACH_ERR_NULL;
    }

    if (target->z < SO101_ARM_WORKSPACE_Z_MIN_M) {
        return SO101_ARM_REACH_ERR_Z_LOW;
    }
    if (target->z > SO101_ARM_WORKSPACE_Z_MAX_M) {
        return SO101_ARM_REACH_ERR_Z_HIGH;
    }
    if ((target->x < SO101_ARM_WORKSPACE_X_MIN_M) ||
        (target->x > SO101_ARM_WORKSPACE_X_MAX_M) ||
        (target->y < SO101_ARM_WORKSPACE_Y_MIN_M) ||
        (target->y > SO101_ARM_WORKSPACE_Y_MAX_M)) {
        return SO101_ARM_REACH_ERR_XYZ_RANGE;
    }

    r = sqrtf((target->x * target->x) + (target->y * target->y));
    if ((r < SO101_ARM_WORKSPACE_R_MIN_M) ||
        (r > SO101_ARM_WORKSPACE_R_MAX_M)) {
        return SO101_ARM_REACH_ERR_XYZ_RANGE;
    }

    return SO101_ARM_REACH_OK;
}

static void so101_arm_init_reach_result(so101_arm_reach_result_t *result)
{
    if (result != 0) {
        memset(result, 0, sizeof(*result));
        result->status = SO101_ARM_REACH_ERR_IK;
    }
}

static void so101_arm_store_reach_result(so101_arm_reach_result_t *result,
                                         so101_arm_reach_status_t status,
                                         const float joint_rad[SO101_ACTIVE_JOINT_COUNT],
                                         const robot_ik_info_t *ik_info,
                                         float limit_margin_rad)
{
    if (result == 0) {
        return;
    }

    result->status = status;
    result->limit_margin_rad = limit_margin_rad;
    if (joint_rad != 0) {
        memcpy(result->joint_rad, joint_rad, sizeof(result->joint_rad));
    }
    if (ik_info != 0) {
        result->position_error_m = ik_info->position_error_m;
        result->pitch_error_rad = ik_info->pitch_error_rad;
        result->side_error_m = ik_info->side_error_m;
        result->ik_iterations = ik_info->iterations;
    }
}

static void so101_arm_servo_apply_defaults(sts_servo_t *servo, sts_servo_bus_t *bus)
{
    if (servo == 0) {
        return;
    }

    servo->bus = bus;
    servo->target_position = servo->position_zero;
    servo->target_angle_rad = 0.0f;
    servo->current_position = servo->position_zero;
    servo->current_angle_rad = 0.0f;
    servo->target_dirty = 0U;
    servo->feedback_dirty = 0U;
    servo->is_initialized = 1U;
    servo->is_online = 0U;
    servo->torque_enabled = 0U;
    servo->protection_enabled = 0U;
    servo->protection_active = 0U;
    servo->max_temperature = SO101_ARM_SERVO_DEFAULT_MAX_TEMPERATURE;
    servo->min_voltage = STS_SERVO_MIN_VOLTAGE_DEFAULT;
    servo->max_voltage = SO101_ARM_SERVO_DEFAULT_MAX_VOLTAGE;
    servo->max_load = 0U;
    servo->max_current = 0U;
    servo->max_position_error = SO101_ARM_SERVO_DEFAULT_POSITION_ERROR;
    servo->fault_flags = STS_SERVO_FAULT_NONE;
    servo->last_status = STS_SERVO_OK;
}

void so101_arm_servo_group_init(void *ctx)
{
    so101_arm_servo_group_t *group = (so101_arm_servo_group_t *)ctx;

    if ((group == 0) || (group->bus == 0) || (group->servos == 0)) {
        return;
    }

    sts_servo_bus_init(group->bus, group->uart);
    group->bus->timeout_ms = group->timeout_ms;

    for (uint8_t i = 0U; i < group->servo_count; i++) {
        so101_arm_servo_apply_defaults(&group->servos[i], group->bus);
    }
}

void so101_arm_servo_group_enable_torque(void *ctx, uint8_t enable)
{
    so101_arm_servo_group_t *group = (so101_arm_servo_group_t *)ctx;
    uint8_t value;

    if ((group == 0) || (group->bus == 0) || (group->servos == 0)) {
        return;
    }

    value = (enable == 0U) ? STS_SERVO_TORQUE_DISABLE : STS_SERVO_TORQUE_ENABLE;

    if (sts_servo_write_mem(group->bus,
                            STS_SERVO_BROADCAST_ID,
                            STS_SERVO_ADDR_TORQUE_ENABLE,
                            &value,
                            1U) == STS_SERVO_OK) {
        for (uint8_t i = 0U; i < group->servo_count; i++) {
            group->servos[i].torque_enabled = (enable == 0U) ? 0U : 1U;
        }
    }
}

static void so101_arm_limit_joints(float joint_rad[SO101_ACTIVE_JOINT_COUNT])
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        joint_rad[i] = so101_arm_clampf(joint_rad[i], so101_arm_joint_min[i], so101_arm_joint_max[i]);
    }
}

static uint8_t so101_arm_joints_within_limits(const float joint_rad[SO101_ACTIVE_JOINT_COUNT])
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if ((joint_rad[i] < so101_arm_joint_min[i]) ||
            (joint_rad[i] > so101_arm_joint_max[i])) {
            return 0U;
        }
    }

    return 1U;
}

static void so101_arm_read_joints(so101_arm_t *arm)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (arm->joint[i].get_angle_rad != 0) {
            arm->current_joint_rad[i] = arm->joint[i].get_angle_rad(arm->joint[i].ctx);
        }
    }
}

static void so101_arm_update_current_position(so101_arm_t *arm)
{
    mat4f_t tip_t;

    if (so101_fk_compute(arm->current_joint_rad, &tip_t) == ROBOT_OK) {
        arm->current_position.x = tip_t.m[0][3];
        arm->current_position.y = tip_t.m[1][3];
        arm->current_position.z = tip_t.m[2][3];
    }
}

static void so101_arm_collect_faults(so101_arm_t *arm)
{
    uint32_t faults = 0UL;
    uint32_t fatal_faults;

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (arm->joint[i].get_faults != 0) {
            faults |= arm->joint[i].get_faults(arm->joint[i].ctx);
        }
    }
    if (arm->gripper.get_faults != 0) {
        faults |= arm->gripper.get_faults(arm->gripper.ctx);
    }

    arm->fault_flags = faults;
    fatal_faults = faults & ~arm->nonfatal_joint_fault_mask;
    arm->fatal_fault_flags = fatal_faults;
    if (fatal_faults != 0UL) {
        arm->state = SO101_ARM_STATE_FAULT;
    }
}

static void so101_arm_step_command_joints(so101_arm_t *arm)
{
    float max_step = arm->max_joint_step_rad;

    if (max_step <= 0.0f) {
        memcpy(arm->command_joint_rad, arm->target_joint_rad, sizeof(arm->command_joint_rad));
        return;
    }

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        float err = arm->target_joint_rad[i] - arm->command_joint_rad[i];

        if (err > max_step) {
            err = max_step;
        } else if (err < -max_step) {
            err = -max_step;
        }
        arm->command_joint_rad[i] += err;
    }
}

static void so101_arm_write_command_joints(so101_arm_t *arm)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (arm->joint[i].set_angle_rad != 0) {
            arm->joint[i].set_angle_rad(arm->joint[i].ctx, arm->command_joint_rad[i]);
        }
    }
}

static void so101_arm_update_joint_io(so101_arm_t *arm)
{
    if (arm->joint_group.update != 0) {
        arm->joint_group.update(arm->joint_group.ctx);
        return;
    }

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (arm->joint[i].update != 0) {
            arm->joint[i].update(arm->joint[i].ctx);
        }
    }
}

static uint8_t so101_arm_joint_target_reached(const so101_arm_t *arm)
{
    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (so101_arm_absf(arm->target_joint_rad[i] - arm->current_joint_rad[i]) >
            arm->reached_angle_eps_rad) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t so101_arm_position_target_reached(const so101_arm_t *arm)
{
    float dx = arm->target_position.x - arm->current_position.x;
    float dy = arm->target_position.y - arm->current_position.y;
    float dz = arm->target_position.z - arm->current_position.z;
    float err = sqrtf(dx * dx + dy * dy + dz * dz);

    return (err <= arm->reached_pos_eps_m) ? 1U : 0U;
}

void so101_arm_init(so101_arm_t *arm)
{
    if (arm == 0) {
        return;
    }

    if (arm->max_joint_step_rad == 0.0f) {
        arm->max_joint_step_rad = SO101_ARM_MAX_JOINT_STEP_RAD;
    }
    if (arm->reached_angle_eps_rad == 0.0f) {
        arm->reached_angle_eps_rad = SO101_ARM_REACHED_ANGLE_EPS_RAD;
    }
    if (arm->reached_pos_eps_m == 0.0f) {
        arm->reached_pos_eps_m = SO101_ARM_REACHED_POS_EPS_M;
    }
    so101_ik_default_options(&arm->ik_options);

    if (arm->joint_group.init != 0) {
        arm->joint_group.init(arm->joint_group.ctx);
    }

    for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
        if (arm->joint[i].init != 0) {
            arm->joint[i].init(arm->joint[i].ctx);
        }
    }
    if (arm->gripper.init != 0) {
        arm->gripper.init(arm->gripper.ctx);
    }

    so101_arm_read_joints(arm);
    so101_arm_limit_joints(arm->current_joint_rad);
    memcpy(arm->target_joint_rad, arm->current_joint_rad, sizeof(arm->target_joint_rad));
    memcpy(arm->command_joint_rad, arm->current_joint_rad, sizeof(arm->command_joint_rad));
    so101_arm_update_current_position(arm);
    arm->target_position = arm->current_position;
    arm->target_pose.x = arm->current_position.x;
    arm->target_pose.y = arm->current_position.y;
    arm->target_pose.z = arm->current_position.z;
    arm->target_pose.roll = arm->current_joint_rad[SO101_JOINT_WRIST_ROLL];
    arm->target_pose.pitch = 0.0f;
    arm->mode = SO101_ARM_MODE_JOINT;
    arm->state = SO101_ARM_STATE_IDLE;
    arm->fault_flags = 0UL;
    arm->fatal_fault_flags = 0UL;
    arm->target_dirty = 0U;
    arm->is_initialized = 1U;
    arm->last_robot_status = ROBOT_OK;
}

so101_arm_status_t so101_arm_set_joint_angles(so101_arm_t *arm,
                                              const float joint_rad[SO101_ACTIVE_JOINT_COUNT])
{
    if ((arm == 0) || (joint_rad == 0)) {
        return SO101_ARM_ERR_NULL;
    }

    memcpy(arm->target_joint_rad, joint_rad, sizeof(arm->target_joint_rad));
    so101_arm_limit_joints(arm->target_joint_rad);
    arm->mode = SO101_ARM_MODE_JOINT;
    arm->state = SO101_ARM_STATE_MOVING;
    arm->target_dirty = 1U;
    return SO101_ARM_OK;
}

so101_arm_status_t so101_arm_set_position(so101_arm_t *arm,
                                          const so101_arm_position_t *position)
{
    if ((arm == 0) || (position == 0)) {
        return SO101_ARM_ERR_NULL;
    }

    arm->target_position = *position;
    arm->target_pose.x = position->x;
    arm->target_pose.y = position->y;
    arm->target_pose.z = position->z;
    arm->mode = SO101_ARM_MODE_CARTESIAN;
    arm->state = SO101_ARM_STATE_MOVING;
    arm->target_dirty = 1U;
    return SO101_ARM_OK;
}

so101_arm_status_t so101_arm_set_pose(so101_arm_t *arm,
                                      const so101_arm_pose_t *pose)
{
    if ((arm == 0) || (pose == 0)) {
        return SO101_ARM_ERR_NULL;
    }

    arm->target_pose = *pose;
    arm->target_position.x = pose->x;
    arm->target_position.y = pose->y;
    arm->target_position.z = pose->z;
    arm->mode = SO101_ARM_MODE_POSE;
    arm->state = SO101_ARM_STATE_MOVING;
    arm->target_dirty = 1U;
    return SO101_ARM_OK;
}

void so101_arm_enable_torque(so101_arm_t *arm, uint8_t enable)
{
    if (arm == 0) {
        return;
    }

    if (arm->joint_group.enable_torque != 0) {
        arm->joint_group.enable_torque(arm->joint_group.ctx, enable);
    } else {
        for (uint8_t i = 0U; i < SO101_ACTIVE_JOINT_COUNT; i++) {
            if (arm->joint[i].enable_torque != 0) {
                arm->joint[i].enable_torque(arm->joint[i].ctx, enable);
            }
        }
        if (arm->gripper.enable_torque != 0) {
            arm->gripper.enable_torque(arm->gripper.ctx, enable);
        }
    }

    if (enable != 0U) {
        arm->fault_flags = 0UL;
        arm->fatal_fault_flags = 0UL;
        arm->state = SO101_ARM_STATE_IDLE;
    }
}

void so101_arm_set_gripper_angle(so101_arm_t *arm, float gripper_rad)
{
    if ((arm == 0) || (arm->gripper.set_angle_rad == 0)) {
        return;
    }

    arm->gripper.set_angle_rad(arm->gripper.ctx, gripper_rad);
}

float so101_arm_get_gripper_angle(const so101_arm_t *arm)
{
    if ((arm == 0) || (arm->gripper.get_angle_rad == 0)) {
        return 0.0f;
    }

    return arm->gripper.get_angle_rad(arm->gripper.ctx);
}

so101_arm_status_t so101_arm_update(so101_arm_t *arm, float dt)
{
    (void)dt;

    if (arm == 0) {
        return SO101_ARM_ERR_NULL;
    }
    if (arm->is_initialized == 0U) {
        return SO101_ARM_ERR_PARAM;
    }

    so101_arm_read_joints(arm);
    so101_arm_update_current_position(arm);
    so101_arm_collect_faults(arm);
    if (arm->fatal_fault_flags != 0UL) {
        so101_arm_update_joint_io(arm);
        return SO101_ARM_ERR_FAULT;
    }

    if (((arm->mode == SO101_ARM_MODE_CARTESIAN) ||
         (arm->mode == SO101_ARM_MODE_POSE)) &&
        (arm->target_dirty != 0U)) {
        vec3f_t target = {
            arm->target_position.x,
            arm->target_position.y,
            arm->target_position.z,
        };
        float ik_joint_rad[SO101_ACTIVE_JOINT_COUNT];
        robot_status_t robot_status;

        if (arm->mode == SO101_ARM_MODE_POSE) {
            robot_pose_target_t pose_target;

            pose_target.position.x = arm->target_pose.x;
            pose_target.position.y = arm->target_pose.y;
            pose_target.position.z = arm->target_pose.z;
            pose_target.roll_rad = arm->target_pose.roll;
            pose_target.pitch_rad = arm->target_pose.pitch;
            robot_status = so101_pose_ik(&pose_target,
                                         arm->current_joint_rad,
                                         &arm->ik_options,
                                         ik_joint_rad,
                                         &arm->ik_info);
        } else {
            robot_status = so101_position_ik(&target,
                                             arm->current_joint_rad,
                                             &arm->ik_options,
                                             ik_joint_rad,
                                             &arm->ik_info);
        }
        arm->last_robot_status = robot_status;
        if (robot_status != ROBOT_OK) {
            arm->state = SO101_ARM_STATE_MOVING;
            so101_arm_update_joint_io(arm);
            return SO101_ARM_ERR_IK;
        }
        if (so101_arm_joints_within_limits(ik_joint_rad) == 0U) {
            arm->last_robot_status = ROBOT_ERR_RANGE;
            arm->ik_info.status = ROBOT_ERR_RANGE;
            arm->state = SO101_ARM_STATE_MOVING;
            so101_arm_update_joint_io(arm);
            return SO101_ARM_ERR_IK;
        }
        memcpy(arm->target_joint_rad, ik_joint_rad, sizeof(arm->target_joint_rad));
        arm->target_dirty = 0U;
    }

    so101_arm_step_command_joints(arm);
    so101_arm_write_command_joints(arm);
    so101_arm_update_joint_io(arm);

    if (so101_arm_is_reached(arm) != 0U) {
        arm->state = SO101_ARM_STATE_REACHED;
    } else {
        arm->state = SO101_ARM_STATE_MOVING;
    }

    return SO101_ARM_OK;
}

uint8_t so101_arm_is_reached(const so101_arm_t *arm)
{
    if (arm == 0) {
        return 0U;
    }
    if (arm->mode == SO101_ARM_MODE_CARTESIAN) {
        return so101_arm_position_target_reached(arm);
    }
    return so101_arm_joint_target_reached(arm);
}

uint32_t so101_arm_get_faults(const so101_arm_t *arm)
{
    return (arm != 0) ? arm->fault_flags : 0UL;
}

so101_arm_reach_status_t so101_arm_check_reachable(so101_arm_t *arm,
                                                   const so101_arm_position_t *target,
                                                   so101_arm_reach_result_t *result)
{
    const float *seed_list[2];
    vec3f_t target_xyz;
    so101_arm_reach_status_t bounds_status;
    so101_arm_reach_status_t best_status = SO101_ARM_REACH_ERR_IK;
    float best_joint_rad[SO101_ACTIVE_JOINT_COUNT] = {0.0f};
    robot_ik_info_t best_info = {0};
    float best_margin = -1000.0f;

    best_info.status = ROBOT_ERR_NO_CONVERGE;
    best_info.position_error_m = 1000.0f;

    so101_arm_init_reach_result(result);

    if ((arm == 0) || (target == 0)) {
        so101_arm_store_reach_result(result, SO101_ARM_REACH_ERR_NULL, 0, 0, 0.0f);
        return SO101_ARM_REACH_ERR_NULL;
    }

    bounds_status = so101_arm_check_workspace_bounds(target);
    if (bounds_status != SO101_ARM_REACH_OK) {
        so101_arm_store_reach_result(result, bounds_status, 0, 0, 0.0f);
        return bounds_status;
    }

    target_xyz.x = target->x;
    target_xyz.y = target->y;
    target_xyz.z = target->z;
    seed_list[0] = arm->current_joint_rad;
    seed_list[1] = so101_arm_home_seed;

    for (uint8_t i = 0U; i < 2U; i++) {
        float candidate_joint_rad[SO101_ACTIVE_JOINT_COUNT];
        robot_ik_info_t ik_info = {0};
        robot_status_t robot_status;
        float limit_margin;
        so101_arm_reach_status_t candidate_status;
        uint8_t candidate_is_better = 0U;

        ik_info.status = ROBOT_ERR_NO_CONVERGE;
        ik_info.position_error_m = 1000.0f;
        memcpy(candidate_joint_rad, seed_list[i], sizeof(candidate_joint_rad));
        robot_status = so101_position_ik(&target_xyz, seed_list[i], &arm->ik_options,
                                         candidate_joint_rad, &ik_info);
        if ((robot_status != ROBOT_OK) ||
            (ik_info.position_error_m > SO101_ARM_REACH_IK_TOLERANCE_M)) {
            candidate_status = SO101_ARM_REACH_ERR_IK;
        } else {
            limit_margin = so101_arm_joint_limit_margin(candidate_joint_rad);
            if (limit_margin < SO101_ARM_REACH_LIMIT_MARGIN_RAD) {
                candidate_status = SO101_ARM_REACH_ERR_LIMIT_MARGIN;
            } else {
                candidate_status = SO101_ARM_REACH_OK;
            }
        }

        limit_margin = so101_arm_joint_limit_margin(candidate_joint_rad);
        if (best_status != SO101_ARM_REACH_OK) {
            candidate_is_better = (candidate_status == SO101_ARM_REACH_OK) ? 1U : 0U;
        } else if ((candidate_status == SO101_ARM_REACH_OK) &&
                   (limit_margin > best_margin)) {
            candidate_is_better = 1U;
        }

        if ((candidate_is_better != 0U) ||
            ((best_status == SO101_ARM_REACH_ERR_IK) &&
             (ik_info.position_error_m < best_info.position_error_m))) {
            best_status = candidate_status;
            memcpy(best_joint_rad, candidate_joint_rad, sizeof(best_joint_rad));
            best_info = ik_info;
            best_margin = limit_margin;
        }
    }

    so101_arm_store_reach_result(result, best_status, best_joint_rad, &best_info, best_margin);
    return best_status;
}
