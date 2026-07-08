#ifndef K1MUSE_CONTROL_CORE__PLACEMENT_PLANNER_H_
#define K1MUSE_CONTROL_CORE__PLACEMENT_PLANNER_H_

#include "k1muse_control_core/grasp_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} k1_point3f_t;

typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
} k1_pose5f_t;

typedef struct {
    float arm_mount_x;
    float arm_mount_y;
    float arm_base_z_at_lift_zero;
    float lift_min_z;
    float lift_max_z;
} k1_robot_geometry_t;

typedef struct {
    float chassis_delta_x;
    float chassis_delta_y;
    float chassis_direction;
    float chassis_distance;
    float lift_target_z;
    k1_pose5f_t grasp_pose;
    k1_pose5f_t pregrasp_pose;
    k1_pose5f_t lifted_pose;
    k1_pose5f_t retreat_pose;
} k1_pickup_plan_t;

typedef struct {
    float chassis_delta_x;
    float chassis_delta_y;
    float chassis_direction;
    float chassis_distance;
    float lift_target_z;
    k1_pose5f_t preplace_pose;
    k1_pose5f_t approach_pose;
    k1_pose5f_t place_pose;
    k1_pose5f_t retreat_pose;
} k1_place_plan_t;

typedef enum {
    K1_PLAN_OK = 0,
    K1_PLAN_NULL,
    K1_PLAN_INVALID_INPUT,
    K1_PLAN_LIFT_OUT_OF_RANGE,
} k1_plan_status_t;

k1_plan_status_t k1_plan_pickup(
    const k1_point3f_t *target_in_chassis,
    const k1_robot_geometry_t *geometry,
    const k1_grasp_variant_t *profile,
    k1_pickup_plan_t *plan);

k1_plan_status_t k1_plan_place(
    const k1_point3f_t *target_in_chassis,
    const k1_robot_geometry_t *geometry,
    const k1_grasp_variant_t *profile,
    k1_place_plan_t *plan);

#ifdef __cplusplus
}
#endif

#endif
