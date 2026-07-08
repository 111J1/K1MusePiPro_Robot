#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "k1muse_control_core/actuator_protocol.h"
#include "k1muse_control_core/control_protocol.h"
#include "k1muse_control_core/grasp_profile.h"
#include "k1muse_control_core/pick_place_task.h"
#include "k1muse_control_core/placement_planner.h"

#define EPSILON (1.0e-5f)

static void test_crc_and_frame_round_trip(void)
{
    static const uint8_t check_data[] = "123456789";
    k1_ctrl_frame_t source;
    k1_ctrl_frame_t decoded;
    k1_ctrl_parser_t parser;
    uint8_t bytes[K1_CTRL_MAX_FRAME_SIZE];
    size_t size;
    size_t i;
    int received = 0;

    assert(k1_crc8_atm(check_data, 9U) == 0xF4U);
    k1_build_arm_pose_frame(&source, 17U, 0.2f, -0.1f, 0.15f, 0.0f, 1.0f);
    size = k1_ctrl_encode_frame(&source, bytes, sizeof(bytes));
    assert(size == 28U);

    k1_ctrl_parser_init(&parser);
    for (i = 0U; i < size; ++i) {
        received += k1_ctrl_parser_input(&parser, bytes[i], &decoded);
    }
    assert(received == 1);
    assert(decoded.target == K1_CTRL_TARGET_ARM);
    assert(decoded.cmd == K1_ARM_CMD_MOVE_POSE);
    assert(decoded.seq == 17U);
    assert(fabsf(k1_get_f32_le(&decoded.payload[0]) - 0.2f) < EPSILON);

    bytes[size - 1U] ^= 0x01U;
    k1_ctrl_parser_init(&parser);
    received = 0;
    for (i = 0U; i < size; ++i) {
        received += k1_ctrl_parser_input(&parser, bytes[i], &decoded);
    }
    assert(received == 0);
}

static k1_grasp_variant_t make_profile(void)
{
    k1_grasp_variant_t profile;
    memset(&profile, 0, sizeof(profile));
    profile.mode = K1_GRASP_MODE_TOP;
    profile.gripper_strategy = K1_GRIPPER_FIXED_ANGLE;
    profile.gripper_open_rad = 1.0f;
    profile.gripper_close_rad = 0.4f;
    profile.pitch_rad = 1.57079632679f;
    profile.preferred_arm_x = 0.20f;
    profile.preferred_arm_z = 0.15f;
    profile.approach_distance_m = 0.08f;
    profile.retreat_distance_m = 0.08f;
    profile.lift_distance_m = 0.05f;
    profile.transport_max_v = 0.2f;
    profile.transport_max_omega = 0.5f;
    profile.settle_time_ms = 500U;
    profile.calibrated = 1U;
    return profile;
}

static void test_profile_and_planner(void)
{
    k1_grasp_variant_t profile = make_profile();
    k1_robot_geometry_t geometry = {0.030406f, 0.0f, 0.222f, 0.0f, 0.50f};
    k1_point3f_t target = {1.0f, 0.10f, 0.60f};
    k1_pickup_plan_t plan;
    k1_place_plan_t place_plan;

    assert(k1_grasp_profile_validate(&profile, -0.167f, 1.888f) == K1_PROFILE_OK);
    assert(k1_plan_pickup(&target, &geometry, &profile, &plan) == K1_PLAN_OK);
    assert(fabsf(plan.chassis_delta_x - 0.769594f) < EPSILON);
    assert(fabsf(plan.chassis_delta_y - 0.10f) < EPSILON);
    assert(fabsf(plan.lift_target_z - 0.228f) < EPSILON);
    assert(fabsf(plan.pregrasp_pose.z - 0.23f) < EPSILON);
    assert(fabsf(plan.lifted_pose.z - 0.20f) < EPSILON);

    assert(k1_plan_place(&target, &geometry, &profile, &place_plan) == K1_PLAN_OK);
    assert(fabsf(place_plan.lift_target_z - plan.lift_target_z) < EPSILON);
    assert(fabsf(place_plan.preplace_pose.x - plan.retreat_pose.x) < EPSILON);
    assert(fabsf(place_plan.preplace_pose.z - plan.retreat_pose.z) < EPSILON);
    assert(fabsf(place_plan.approach_pose.z - plan.lifted_pose.z) < EPSILON);
    assert(fabsf(place_plan.place_pose.x - plan.grasp_pose.x) < EPSILON);
    assert(fabsf(place_plan.place_pose.z - plan.grasp_pose.z) < EPSILON);
    assert(fabsf(place_plan.retreat_pose.z - plan.retreat_pose.z) < EPSILON);

    profile.calibrated = 0U;
    assert(k1_grasp_profile_validate(&profile, -0.167f, 1.888f) ==
           K1_PROFILE_NOT_CALIBRATED);
}

static void test_pick_place_state_machine(void)
{
    k1_grasp_variant_t profile = make_profile();
    k1_pickup_plan_t plan;
    k1_demo_route_t route;
    k1_pick_place_task_t task;
    k1_task_action_t action;
    int guard = 0;

    memset(&plan, 0, sizeof(plan));
    plan.chassis_direction = 0.1f;
    plan.chassis_distance = 0.5f;
    plan.lift_target_z = 0.2f;
    plan.grasp_pose = (k1_pose5f_t){0.2f, 0.0f, 0.15f, 0.0f, 1.5707963f};
    plan.pregrasp_pose = (k1_pose5f_t){0.2f, 0.0f, 0.23f, 0.0f, 1.5707963f};
    plan.lifted_pose = (k1_pose5f_t){0.2f, 0.0f, 0.20f, 0.0f, 1.5707963f};

    memset(&route, 0, sizeof(route));
    route.pickup_chassis_speed = 0.15f;
    route.transport_distance = 1.0f;
    route.transport_speed = 0.15f;
    route.drop_lift_z = 0.1f;
    route.carry_pose = (k1_pose5f_t){0.16f, 0.0f, 0.22f, 0.0f, 0.0f};
    route.preplace_pose = (k1_pose5f_t){0.20f, 0.0f, 0.23f, 0.0f, 1.5707963f};
    route.place_pose = (k1_pose5f_t){0.20f, 0.0f, 0.15f, 0.0f, 1.5707963f};
    route.retreat_pose = route.preplace_pose;
    route.release_settle_ms = 300U;

    k1_pick_place_task_init(&task);
    assert(k1_pick_place_task_start(&task, &plan, &profile, &route, &action));
    assert(action.type == K1_ACTION_STOP_ALL);

    while ((task.state != K1_TASK_COMPLETED) && (guard++ < 32)) {
        int event = (action.type == K1_ACTION_WAIT) ?
                    K1_TASK_EVENT_TIMER_COMPLETED :
                    K1_TASK_EVENT_COMMAND_COMPLETED;
        assert(k1_pick_place_task_handle_event(&task, event, &action));
    }
    assert(task.state == K1_TASK_COMPLETED);
    assert(action.type == K1_ACTION_TASK_COMPLETED);

    k1_pick_place_task_init(&task);
    assert(k1_pick_place_task_start(&task, &plan, &profile, &route, &action));
    assert(k1_pick_place_task_handle_event(&task, K1_TASK_EVENT_TIMEOUT, &action));
    assert(task.state == K1_TASK_FAILED);
    assert(action.type == K1_ACTION_STOP_ALL);
}

int main(void)
{
    test_crc_and_frame_round_trip();
    test_profile_and_planner();
    test_pick_place_state_machine();
    puts("k1muse_control_core tests passed");
    return 0;
}
