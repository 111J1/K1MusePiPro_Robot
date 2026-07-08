# Control Manager API Contract

## Action Entry

The formal upper-computer entry remains:

```text
k1muse_manager_msgs/action/ExecuteTask
```

The control manager owns actuator execution only. Vision, speech, and task policy should provide a resolved object, variant, and target height.

## Formal Task Types

```text
find
pick
place
pick_place
stop
lift
move
rotate
```

Compatibility/debug task types:

```text
grasp    alias of pick
release  debug/manual gripper-open only
```

`release` is not the formal place task. A formal place task must include lift, arm approach, gripper open, settle, and retreat.

## Goal Fields

```text
string task_type
string target_class
string target_id
float32 target_z
float32 place_z
bool hold
builtin_interfaces/Duration timeout
```

Field semantics:

```text
target_class  bottle / box / umbrella
target_id     side / top, empty means default variant
target_z      pick TCP height for pick; place TCP height for place
place_z       place TCP height for pick_place
hold          legacy field; pick always ends holding at carry pose
timeout       whole-task timeout
```

## Pick Contract

Input example:

```text
task_type = "pick"
target_class = "umbrella"
target_id = "top"
target_z = 0.350
```

Behavior:

```text
STOP_ALL
ENSURE_ARM_READY
ENSURE_LIFT_READY
LIFT_MOVE_Z
GRIPPER_OPEN
ARM_MOVE_PREGRASP
ARM_MOVE_GRASP
GRIPPER_CLOSE
WAIT_SETTLE
ARM_MOVE_CARRY
DONE_HOLDING
```

Important: pick ends at carry pose while keeping the object clamped. It must not release.

## Place Contract

Input example:

```text
task_type = "place"
target_class = "umbrella"
target_id = "top"
target_z = 0.350
```

Place planning must be derived from the same pick profile. No independent place parameters are introduced in the first engineering version.

Reverse mapping:

```text
place_pose    = pickup.grasp_pose
approach_pose = pickup.lifted_pose
preplace_pose = pickup.retreat_pose
retreat_pose  = pickup.retreat_pose
lift_target_z = pickup.lift_target_z for the place target_z
```

Behavior:

```text
STOP_ALL
ENSURE_ARM_READY
ENSURE_LIFT_READY
LIFT_MOVE_Z
ARM_MOVE_PREPLACE
ARM_MOVE_APPROACH
ARM_MOVE_PLACE
GRIPPER_OPEN
WAIT_SETTLE
ARM_MOVE_RETREAT
ARM_MOVE_CARRY
DONE_RELEASED
```

## Pick-Place Contract

Input example:

```text
task_type = "pick_place"
target_class = "umbrella"
target_id = "top"
target_z = 0.350
place_z = 0.450
```

Behavior:

```text
pick(target_z)
place(place_z)
```

`place_z` is required for `pick_place`. This avoids overloading one height field with two physical meanings.

## Stop Contract

```text
task_type = "stop"
```

Behavior:

```text
STOP chassis
STOP lift
STOP arm
```

STOP must be valid in any state and should release lower-computer arbitration.

## Lift Contract

```text
task_type = "lift"
target_class = absolute lift z in meters
```

Behavior:

```text
ENSURE_LIFT_READY
LIFT_MOVE_Z
```

If lift has a fault:

```text
LIFT_CLEAR_FAULT
LIFT_HOME
LIFT_MOVE_Z
```

## Result Semantics

Success:

```text
success = true
final_state = 2
reason = "completed"
```

Failure:

```text
success = false
final_state = 3
reason = concrete error text
```

Cancel:

```text
success = false
final_state = 4
reason = "Task cancelled"
```

## Execution Policy

`demo_tolerant`:

```text
gripper timeout without arm fault -> success with warning
arm pose timeout/failed with xyz_error <= 0.015 m -> success with warning
lift fault -> fail current command, recover through clear + home before next movement
```

`strict`:

```text
only matching COMPLETED RESULT is success
```
