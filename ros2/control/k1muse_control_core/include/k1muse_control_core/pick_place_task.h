#ifndef K1MUSE_CONTROL_CORE__PICK_PLACE_TASK_H_
#define K1MUSE_CONTROL_CORE__PICK_PLACE_TASK_H_

#include <stddef.h>
#include <stdint.h>

#include "k1muse_control_core/grasp_profile.h"
#include "k1muse_control_core/placement_planner.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    K1_TASK_IDLE = 0,
    K1_TASK_PRECHECK,
    K1_TASK_ARM_SAFE,
    K1_TASK_LIFT_HOME,
    K1_TASK_MOVE_CHASSIS_PICKUP,
    K1_TASK_MOVE_LIFT_PICKUP,
    K1_TASK_OPEN_GRIPPER,
    K1_TASK_MOVE_PREGRASP,
    K1_TASK_MOVE_GRASP,
    K1_TASK_CLOSE_GRIPPER,
    K1_TASK_SETTLE_GRASP,
    K1_TASK_TEST_LIFT,
    K1_TASK_MOVE_CARRY,
    K1_TASK_TRANSPORT,
    K1_TASK_MOVE_LIFT_DROP,
    K1_TASK_MOVE_PREPLACE,
    K1_TASK_MOVE_PLACE,
    K1_TASK_OPEN_RELEASE,
    K1_TASK_SETTLE_RELEASE,
    K1_TASK_RETREAT_PLACE,
    K1_TASK_COMPLETED,
    K1_TASK_FAILED,
    K1_TASK_CANCELLED,
} k1_pick_place_state_t;

typedef enum {
    K1_TASK_EVENT_COMMAND_COMPLETED = 0,
    K1_TASK_EVENT_TIMER_COMPLETED,
    K1_TASK_EVENT_COMMAND_FAILED,
    K1_TASK_EVENT_TIMEOUT,
    K1_TASK_EVENT_CANCEL,
} k1_task_event_t;

typedef enum {
    K1_ACTION_NONE = 0,
    K1_ACTION_STOP_ALL,
    K1_ACTION_ARM_HOME,
    K1_ACTION_CHASSIS_MOVE_DISTANCE,
    K1_ACTION_LIFT_MOVE_Z,
    K1_ACTION_ARM_MOVE_POSE,
    K1_ACTION_ARM_GRIPPER,
    K1_ACTION_WAIT,
    K1_ACTION_TASK_COMPLETED,
    K1_ACTION_TASK_FAILED,
    K1_ACTION_LIFT_HOME,
} k1_task_action_type_t;

typedef struct {
    int32_t type;
    float values[6];
    uint32_t wait_ms;
} k1_task_action_t;

typedef struct {
    float pickup_chassis_speed;
    float transport_direction;
    float transport_distance;
    float transport_speed;
    float drop_lift_z;
    k1_pose5f_t carry_pose;
    k1_pose5f_t preplace_pose;
    k1_pose5f_t place_pose;
    k1_pose5f_t retreat_pose;
    uint32_t release_settle_ms;
} k1_demo_route_t;

typedef struct {
    int32_t state;
    k1_pickup_plan_t pickup_plan;
    k1_grasp_variant_t profile;
    k1_demo_route_t route;
} k1_pick_place_task_t;

void k1_pick_place_task_init(k1_pick_place_task_t *task);
int k1_pick_place_task_start(k1_pick_place_task_t *task,
                             const k1_pickup_plan_t *pickup_plan,
                             const k1_grasp_variant_t *profile,
                             const k1_demo_route_t *route,
                             k1_task_action_t *action);
int k1_pick_place_task_handle_event(k1_pick_place_task_t *task,
                                    int32_t event,
                                    k1_task_action_t *action);
const char *k1_pick_place_state_name(int32_t state);
size_t k1_pick_place_task_size(void);
int32_t k1_pick_place_task_get_state(const k1_pick_place_task_t *task);

#ifdef __cplusplus
}
#endif

#endif
