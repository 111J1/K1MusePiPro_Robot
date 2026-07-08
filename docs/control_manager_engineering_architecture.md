# Control Manager Engineering Architecture

## Goal

Move the verified PC-side actuator behavior into the existing ROS2 `k1muse_control_manager` framework. Formal handoff should use the manager package, config, and pure C core; PC-side validation tools remain local bring-up and regression tooling in the full development workspace.

## Runtime Shape

```text
control_manager_node
  -> task dispatch
  -> GraspTaskExecutor
  -> DeviceSession
  -> Stm32ActuatorClient
  -> SerialTransport
  -> k1muse_control_core
  -> STM32 lower-computer
```

The default backend is `mock`. The real K1 deployment selects `backend: stm32_uart`.

## Formal Task Model

```text
pick       pick object and end at carry pose, still clamped
place      place currently held object, open gripper, retreat
pick_place pick followed by place
stop       STOP_ALL
lift       lift only
find       existing vision interaction
move/rotate reserved for navigation/chassis integration
```

Compatibility/debug:

```text
grasp   alias of pick
release manual gripper open only
```

`release` is not a formal place task.

## Device Session

Arm home is not repeated when the arm is already ready:

```text
has_fault == false
is_busy == false
state in {IDLE, WAIT_TARGET, REACHED}
```

This preserves the nearest-solution IK behavior during continuous tasks.

Lift home is skipped after it has been homed once and has no fault:

```text
has_fault == false
is_homed == true
```

If lift has fault:

```text
LIFT_CLEAR_FAULT
LIFT_HOME
```

## Pick Flow

```text
STOP_ALL
ENSURE_ARM_READY
ENSURE_LIFT_READY
LIFT_MOVE_Z
GRIPPER_OPEN
PREGRASP
GRASP
GRIPPER_CLOSE
WAIT_SETTLE
CARRY
DONE_HOLDING
```

Pick never releases the object.

## Place Flow

Place is derived from the same pickup profile. No independent place profile is introduced in the first engineering version.

Reverse mapping:

```text
place_pose    = pickup.grasp_pose
approach_pose = pickup.lifted_pose
preplace_pose = pickup.retreat_pose
retreat_pose  = pickup.retreat_pose
lift_target_z = pickup.lift_target_z for the place target_z
```

Flow:

```text
STOP_ALL
ENSURE_ARM_READY
ENSURE_LIFT_READY
LIFT_MOVE_Z
PREPLACE
APPROACH
PLACE
GRIPPER_OPEN
WAIT_SETTLE
RETREAT
CARRY
DONE_RELEASED
```

## Config

Formal config:

```text
k1muse_control_manager/config/control_manager.real.yaml
k1muse_control_manager/config/grasp_profiles.yaml
```

Calibrated objects:

```text
bottle.side
box.side
box.top
umbrella.top
```

The pitch value for top grasps remains `1.5`, not `1.57`.

## Safety

Cancel and exceptions call:

```text
STOP_ALL
```

`demo_tolerant` mode keeps the validated competition-video behavior:

```text
gripper timeout without arm fault -> assumed success
arm pose failure with xyz_error <= 0.015 m -> assumed success
lift fault -> explicit failure, then clear+home before next lift move
```

`strict` mode requires matching completed results.
