#include "k1muse_control_core/pick_place_task.h"

#include <math.h>
#include <string.h>

static void action_clear(k1_task_action_t *action)
{
    memset(action, 0, sizeof(*action));
    action->type = K1_ACTION_NONE;
}

static void action_pose(k1_task_action_t *action, const k1_pose5f_t *pose)
{
    action_clear(action);
    action->type = K1_ACTION_ARM_MOVE_POSE;
    action->values[0] = pose->x;
    action->values[1] = pose->y;
    action->values[2] = pose->z;
    action->values[3] = pose->roll;
    action->values[4] = pose->pitch;
}

static int pose_valid(const k1_pose5f_t *pose)
{
    return (pose != NULL) && isfinite(pose->x) && isfinite(pose->y) &&
           isfinite(pose->z) && isfinite(pose->roll) && isfinite(pose->pitch);
}

static int route_valid(const k1_demo_route_t *route)
{
    return (route != NULL) && isfinite(route->pickup_chassis_speed) &&
           isfinite(route->transport_direction) &&
           isfinite(route->transport_distance) &&
           isfinite(route->transport_speed) &&
           isfinite(route->drop_lift_z) &&
           pose_valid(&route->carry_pose) &&
           pose_valid(&route->preplace_pose) &&
           pose_valid(&route->place_pose) &&
           pose_valid(&route->retreat_pose) &&
           (route->pickup_chassis_speed > 0.0f) &&
           (route->transport_distance >= 0.0f) &&
           (route->transport_speed > 0.0f);
}

void k1_pick_place_task_init(k1_pick_place_task_t *task)
{
    if (task == NULL) {
        return;
    }
    memset(task, 0, sizeof(*task));
    task->state = K1_TASK_IDLE;
}

int k1_pick_place_task_start(k1_pick_place_task_t *task,
                             const k1_pickup_plan_t *pickup_plan,
                             const k1_grasp_variant_t *profile,
                             const k1_demo_route_t *route,
                             k1_task_action_t *action)
{
    if ((task == NULL) || (pickup_plan == NULL) || (profile == NULL) ||
        (!route_valid(route)) || (action == NULL) ||
        (profile->calibrated == 0U) ||
        (route->pickup_chassis_speed > profile->transport_max_v) ||
        (route->transport_speed > profile->transport_max_v)) {
        return 0;
    }
    task->pickup_plan = *pickup_plan;
    task->profile = *profile;
    task->route = *route;
    task->state = K1_TASK_PRECHECK;
    action_clear(action);
    action->type = K1_ACTION_STOP_ALL;
    return 1;
}

static int fail_task(k1_pick_place_task_t *task, int32_t state,
                     k1_task_action_t *action)
{
    task->state = state;
    action_clear(action);
    action->type = K1_ACTION_STOP_ALL;
    return 1;
}

int k1_pick_place_task_handle_event(k1_pick_place_task_t *task,
                                    int32_t event,
                                    k1_task_action_t *action)
{
    if ((task == NULL) || (action == NULL)) {
        return 0;
    }
    action_clear(action);

    if (event == K1_TASK_EVENT_CANCEL) {
        return fail_task(task, K1_TASK_CANCELLED, action);
    }
    if ((event == K1_TASK_EVENT_COMMAND_FAILED) ||
        (event == K1_TASK_EVENT_TIMEOUT)) {
        return fail_task(task, K1_TASK_FAILED, action);
    }

    if (task->state == K1_TASK_SETTLE_GRASP) {
        if (event != K1_TASK_EVENT_TIMER_COMPLETED) {
            return 0;
        }
    } else if (task->state == K1_TASK_SETTLE_RELEASE) {
        if (event != K1_TASK_EVENT_TIMER_COMPLETED) {
            return 0;
        }
    } else if (event != K1_TASK_EVENT_COMMAND_COMPLETED) {
        return 0;
    }

    switch (task->state) {
    case K1_TASK_PRECHECK:
        task->state = K1_TASK_ARM_SAFE;
        action->type = K1_ACTION_ARM_HOME;
        break;

    case K1_TASK_ARM_SAFE:
        task->state = K1_TASK_LIFT_HOME;
        action->type = K1_ACTION_LIFT_HOME;
        break;

    case K1_TASK_LIFT_HOME:
        task->state = K1_TASK_MOVE_CHASSIS_PICKUP;
        action->type = K1_ACTION_CHASSIS_MOVE_DISTANCE;
        action->values[0] = task->pickup_plan.chassis_direction;
        action->values[1] = task->pickup_plan.chassis_distance;
        action->values[2] = task->route.pickup_chassis_speed;
        break;

    case K1_TASK_MOVE_CHASSIS_PICKUP:
        task->state = K1_TASK_MOVE_LIFT_PICKUP;
        action->type = K1_ACTION_LIFT_MOVE_Z;
        action->values[0] = task->pickup_plan.lift_target_z;
        break;

    case K1_TASK_MOVE_LIFT_PICKUP:
        task->state = K1_TASK_OPEN_GRIPPER;
        action->type = K1_ACTION_ARM_GRIPPER;
        action->values[0] = task->profile.gripper_open_rad;
        break;

    case K1_TASK_OPEN_GRIPPER:
        task->state = K1_TASK_MOVE_PREGRASP;
        action_pose(action, &task->pickup_plan.pregrasp_pose);
        break;

    case K1_TASK_MOVE_PREGRASP:
        task->state = K1_TASK_MOVE_GRASP;
        action_pose(action, &task->pickup_plan.grasp_pose);
        break;

    case K1_TASK_MOVE_GRASP:
        task->state = K1_TASK_CLOSE_GRIPPER;
        action->type = K1_ACTION_ARM_GRIPPER;
        action->values[0] = task->profile.gripper_close_rad;
        break;

    case K1_TASK_CLOSE_GRIPPER:
        task->state = K1_TASK_SETTLE_GRASP;
        action->type = K1_ACTION_WAIT;
        action->wait_ms = task->profile.settle_time_ms;
        break;

    case K1_TASK_SETTLE_GRASP:
        task->state = K1_TASK_TEST_LIFT;
        action_pose(action, &task->pickup_plan.lifted_pose);
        break;

    case K1_TASK_TEST_LIFT:
        task->state = K1_TASK_MOVE_CARRY;
        action_pose(action, &task->route.carry_pose);
        break;

    case K1_TASK_MOVE_CARRY:
        task->state = K1_TASK_TRANSPORT;
        action->type = K1_ACTION_CHASSIS_MOVE_DISTANCE;
        action->values[0] = task->route.transport_direction;
        action->values[1] = task->route.transport_distance;
        action->values[2] = task->route.transport_speed;
        break;

    case K1_TASK_TRANSPORT:
        task->state = K1_TASK_MOVE_LIFT_DROP;
        action->type = K1_ACTION_LIFT_MOVE_Z;
        action->values[0] = task->route.drop_lift_z;
        break;

    case K1_TASK_MOVE_LIFT_DROP:
        task->state = K1_TASK_MOVE_PREPLACE;
        action_pose(action, &task->route.preplace_pose);
        break;

    case K1_TASK_MOVE_PREPLACE:
        task->state = K1_TASK_MOVE_PLACE;
        action_pose(action, &task->route.place_pose);
        break;

    case K1_TASK_MOVE_PLACE:
        task->state = K1_TASK_OPEN_RELEASE;
        action->type = K1_ACTION_ARM_GRIPPER;
        action->values[0] = task->profile.gripper_open_rad;
        break;

    case K1_TASK_OPEN_RELEASE:
        task->state = K1_TASK_SETTLE_RELEASE;
        action->type = K1_ACTION_WAIT;
        action->wait_ms = task->route.release_settle_ms;
        break;

    case K1_TASK_SETTLE_RELEASE:
        task->state = K1_TASK_RETREAT_PLACE;
        action_pose(action, &task->route.retreat_pose);
        break;

    case K1_TASK_RETREAT_PLACE:
        task->state = K1_TASK_COMPLETED;
        action->type = K1_ACTION_TASK_COMPLETED;
        break;

    default:
        return 0;
    }
    return 1;
}

const char *k1_pick_place_state_name(int32_t state)
{
    static const char *const names[] = {
        "idle", "precheck", "arm_safe", "lift_home", "move_chassis_pickup",
        "move_lift_pickup", "open_gripper", "move_pregrasp",
        "move_grasp", "close_gripper", "settle_grasp", "test_lift",
        "move_carry", "transport", "move_lift_drop", "move_preplace",
        "move_place", "open_release", "settle_release", "retreat_place",
        "completed", "failed", "cancelled"
    };
    if ((state < K1_TASK_IDLE) || (state > K1_TASK_CANCELLED)) {
        return "unknown";
    }
    return names[state];
}

size_t k1_pick_place_task_size(void)
{
    return sizeof(k1_pick_place_task_t);
}

int32_t k1_pick_place_task_get_state(const k1_pick_place_task_t *task)
{
    return (task == NULL) ? K1_TASK_FAILED : task->state;
}
