#include "k1muse_control_core/placement_planner.h"

#include <math.h>
#include <string.h>

static int value_valid(float value)
{
    return isfinite(value) ? 1 : 0;
}

static void compute_tool_axis(const k1_pose5f_t *pose, float axis[3])
{
    float yaw = atan2f(pose->y, pose->x);
    float pitch_cos = cosf(pose->pitch);
    axis[0] = pitch_cos * cosf(yaw);
    axis[1] = pitch_cos * sinf(yaw);
    axis[2] = -sinf(pose->pitch);
}

k1_plan_status_t k1_plan_pickup(
    const k1_point3f_t *target_in_chassis,
    const k1_robot_geometry_t *geometry,
    const k1_grasp_variant_t *profile,
    k1_pickup_plan_t *plan)
{
    float axis[3];

    if ((target_in_chassis == NULL) || (geometry == NULL) ||
        (profile == NULL) || (plan == NULL)) {
        return K1_PLAN_NULL;
    }
    if ((!value_valid(target_in_chassis->x)) ||
        (!value_valid(target_in_chassis->y)) ||
        (!value_valid(target_in_chassis->z)) ||
        (!value_valid(profile->preferred_arm_x)) ||
        (!value_valid(profile->preferred_arm_y)) ||
        (!value_valid(profile->preferred_arm_z))) {
        return K1_PLAN_INVALID_INPUT;
    }

    memset(plan, 0, sizeof(*plan));
    plan->chassis_delta_x = target_in_chassis->x - geometry->arm_mount_x -
                            profile->preferred_arm_x;
    plan->chassis_delta_y = target_in_chassis->y - geometry->arm_mount_y -
                            profile->preferred_arm_y;
    plan->chassis_direction = atan2f(plan->chassis_delta_y,
                                     plan->chassis_delta_x);
    plan->chassis_distance = hypotf(plan->chassis_delta_x,
                                    plan->chassis_delta_y);
    plan->lift_target_z = target_in_chassis->z -
                          geometry->arm_base_z_at_lift_zero -
                          profile->preferred_arm_z;
    if ((plan->lift_target_z < geometry->lift_min_z - 1e-6f) ||
        (plan->lift_target_z > geometry->lift_max_z + 1e-6f)) {
        return K1_PLAN_LIFT_OUT_OF_RANGE;
    }

    plan->grasp_pose.x = profile->preferred_arm_x;
    plan->grasp_pose.y = profile->preferred_arm_y;
    plan->grasp_pose.z = profile->preferred_arm_z;
    plan->grasp_pose.roll = profile->roll_rad;
    plan->grasp_pose.pitch = profile->pitch_rad;
    compute_tool_axis(&plan->grasp_pose, axis);

    plan->pregrasp_pose = plan->grasp_pose;
    plan->pregrasp_pose.x -= axis[0] * profile->approach_distance_m;
    plan->pregrasp_pose.y -= axis[1] * profile->approach_distance_m;
    plan->pregrasp_pose.z -= axis[2] * profile->approach_distance_m;

    plan->lifted_pose = plan->grasp_pose;
    plan->lifted_pose.z += profile->lift_distance_m;

    plan->retreat_pose = plan->lifted_pose;
    plan->retreat_pose.x -= axis[0] * profile->retreat_distance_m;
    plan->retreat_pose.y -= axis[1] * profile->retreat_distance_m;
    plan->retreat_pose.z -= axis[2] * profile->retreat_distance_m;
    return K1_PLAN_OK;
}

k1_plan_status_t k1_plan_place(
    const k1_point3f_t *target_in_chassis,
    const k1_robot_geometry_t *geometry,
    const k1_grasp_variant_t *profile,
    k1_place_plan_t *plan)
{
    k1_pickup_plan_t pickup_plan;
    k1_plan_status_t status;

    if (plan == NULL) {
        return K1_PLAN_NULL;
    }

    status = k1_plan_pickup(target_in_chassis, geometry, profile, &pickup_plan);
    if (status != K1_PLAN_OK) {
        memset(plan, 0, sizeof(*plan));
        return status;
    }

    memset(plan, 0, sizeof(*plan));
    plan->chassis_delta_x = pickup_plan.chassis_delta_x;
    plan->chassis_delta_y = pickup_plan.chassis_delta_y;
    plan->chassis_direction = pickup_plan.chassis_direction;
    plan->chassis_distance = pickup_plan.chassis_distance;
    plan->lift_target_z = pickup_plan.lift_target_z;
    plan->preplace_pose = pickup_plan.retreat_pose;
    plan->approach_pose = pickup_plan.lifted_pose;
    plan->place_pose = pickup_plan.grasp_pose;
    plan->retreat_pose = pickup_plan.retreat_pose;
    return K1_PLAN_OK;
}
