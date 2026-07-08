#!/usr/bin/env python3
"""PC-side Bluetooth demo console for the EmbodiedAI robot MCU.

Default transport is the HC-05/BT path on COM13. Frames sent through Bluetooth
must declare CTRL_SRC_BT, because the MCU rejects frames whose source byte does
not match the physical UART.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    import serial
except ImportError:  # pragma: no cover - depends on PC environment
    serial = None


SOF = bytes((0xA5, 0x5A))
CTRL_PROTOCOL_MAX_PAYLOAD = 64

CTRL_SRC_BT = 0x01
CTRL_SRC_HOST = 0x02
CTRL_SRC_MCU = 0x10

CTRL_TARGET_SYSTEM = 0x00
CTRL_TARGET_CHASSIS = 0x01
CTRL_TARGET_ARM = 0x02
CTRL_TARGET_LIFT = 0x03
CTRL_TARGET_PERIPHERAL = 0x04

CTRL_SYS_CMD_DEMO_STOP = 0x01
CTRL_SYS_CMD_DEMO_RUN = 0x02
CTRL_SYS_CMD_DEMO_HOME = 0x03
CTRL_SYS_RPT_DEMO_STATUS = 0x80
CTRL_SYS_RPT_DEMO_RESULT = 0x81

CTRL_CHS_CMD_STOP = 0x00
CTRL_CHS_CMD_MOV = 0x01

CTRL_ARM_CMD_STOP = 0x00
CTRL_ARM_CMD_HOME = 0x01
CTRL_ARM_CMD_MOVE_XYZ = 0x02
CTRL_ARM_CMD_MOVE_POSE = 0x03
CTRL_ARM_CMD_GRIPPER = 0x04
CTRL_ARM_RPT_STATUS = 0x80
CTRL_ARM_RPT_RESULT = 0x81

CTRL_LIFT_CMD_STOP = 0x00
CTRL_LIFT_CMD_HOME = 0x01
CTRL_LIFT_CMD_MOVE_Z = 0x02
CTRL_LIFT_RPT_STATUS = 0x80
CTRL_LIFT_RPT_RESULT = 0x81

RESULT_NONE = 0
RESULT_ACCEPTED = 1
RESULT_REJECTED = 2
RESULT_COMPLETED = 3
RESULT_ABORTED = 4
RESULT_FAILED = 5
RESULT_SUPERSEDED = 6

DEMO_ID_STATIC_PICK_PLACE = 0x01
DEMO_ID_LAYER_PICK = 0x02
DEMO_ID_LAYER_PLACE = 0x03
DEMO_ID_LAYER_TRANSFER = 0x04
DEMO_ID_KEY_TURN = 0x05
DEMO_ID_CABINET_PULL = 0x06

DEMO_NAMES = {
    "static": DEMO_ID_STATIC_PICK_PLACE,
    "pick": DEMO_ID_LAYER_PICK,
    "place": DEMO_ID_LAYER_PLACE,
    "transfer": DEMO_ID_LAYER_TRANSFER,
    "key-turn": DEMO_ID_KEY_TURN,
    "cabinet-pull": DEMO_ID_CABINET_PULL,
}

DEMO_VARIANTS = {
    "auto": 0,
    "down": 1,
    "up": 2,
}

DEFAULT_PROFILE_PATH = Path(__file__).with_name("demo_tuning_profile.json")

DEFAULT_PROFILE = {
    "layers": {
        "1": 0.010,
        "2": 0.100,
        "3": 0.360,
    },
    "layers_down": {
        "1": 0.010,
        "2": 0.100,
        "3": 0.360,
    },
    "layers_up": {
        "1": 0.010,
        "2": 0.100,
        "3": 0.380,
    },
    "side_pick_lift_z": 0.210,
    "top": {
        "pick_approach": [0.220, 0.000, 0.120, 0.000, 0.000],
        "pick_down": [0.220, 0.000, 0.060, 0.000, 0.000],
        "pick_lift": [0.220, 0.000, 0.150, 0.000, 0.000],
        "place_approach": [0.220, 0.080, 0.120, 0.000, 0.000],
        "place_down": [0.220, 0.080, 0.060, 0.000, 0.000],
        "place_lift": [0.220, 0.080, 0.150, 0.000, 0.000],
    },
    "side": {
        "safe": [0.240, 0.000, 0.180, 1.5707963, 0.000],
        "pick_approach": [0.350, 0.000, 0.130, 1.5707963, 0.000],
        "pick_clearance": [0.350, 0.000, 0.180, 1.5707963, 0.000],
        "pick_pre_insert": [0.430, 0.000, 0.180, 1.5707963, 0.000],
        "pick_insert": [0.430, 0.000, 0.130, 1.5707963, 0.000],
        "carry_safe": [0.240, 0.000, 0.180, 1.5707963, 0.000],
        "place_approach": [0.350, 0.000, 0.130, 1.5707963, 0.000],
        "place_clearance": [0.350, 0.000, 0.180, 1.5707963, 0.000],
        "place_pre_insert": [0.430, 0.000, 0.180, 1.5707963, 0.000],
        "place_insert": [0.430, 0.000, 0.130, 1.5707963, 0.000],
        "retreat_safe": [0.350, 0.000, 0.130, 1.5707963, 0.000],
    },
    "gripper": {
        "open": 1.30,
        "close": 0.35,
        "close_tolerance": 0.20,
    },
    "cabinet_lift_z_m": 0.168,
    "cabinet_pull_speed_mps": 0.20,
    "cabinet_pull_duration_s": 1.0,
    "key_lift_z_m": 0.168,
    "key_pull_speed_mps": 0.20,
    "key_pull_duration_s": 1.0,
    "key_pull_omega_radps": 0.5,
    "tolerance": {
        "pick_lift_z": 0.05,
    },
}

CONFIG_PATH = Path(__file__).parents[1] / "Application" / "Task" / "demo_macro_config.h"


def crc8_atm(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def encode_frame(src: int, target: int, cmd: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > CTRL_PROTOCOL_MAX_PAYLOAD:
        raise ValueError(f"payload too large: {len(payload)}")
    header = bytes((src & 0xFF, target & 0xFF, cmd & 0xFF, seq & 0xFF, len(payload)))
    body = header + payload
    return SOF + body + bytes((crc8_atm(body),))


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def load_profile(path: Path) -> dict:
    if not path.exists():
        return DEFAULT_PROFILE
    with path.open("r", encoding="utf-8") as file:
        profile = json.load(file)
    return profile


def save_profile(path: Path, profile: dict) -> None:
    path.write_text(json.dumps(profile, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def set_layer_height(profile: dict, path: Path, layer: int, z: float) -> None:
    profile.setdefault("layers", {})[str(layer)] = float(z)
    save_profile(path, profile)
    print(f"layer {layer} = {z:.4f} m")


def set_cabinet_lift_z(profile: dict, path: Path, z: float) -> None:
    profile["cabinet_lift_z_m"] = float(z)
    save_profile(path, profile)
    print(f"cabinet_lift_z = {z:.4f} m")


def set_key_lift_z(profile: dict, path: Path, z: float) -> None:
    profile["key_lift_z_m"] = float(z)
    save_profile(path, profile)
    print(f"key_lift_z = {z:.4f} m")


def set_top_pose(profile: dict, path: Path, name: str, x: float, y: float, z: float, roll: float, pitch: float) -> None:
    profile.setdefault("top", {})[name] = [x, y, z, roll, pitch]
    save_profile(path, profile)
    print(f"top.{name} = [{x:.4f}, {y:.4f}, {z:.4f}, {roll:.4f}, {pitch:.4f}]")


def set_layer_direction_height(profile: dict, path: Path, direction: str, layer: int, z: float) -> None:
    if direction not in ("up", "down"):
        raise ValueError("direction must be up or down")
    key = f"layers_{direction}"
    profile.setdefault(key, {})[str(layer)] = float(z)
    save_profile(path, profile)
    print(f"layer {layer} {direction} = {z:.4f} m")


def set_gripper_preset(profile: dict, path: Path, name: str, rad: float) -> None:
    if name not in ("open", "close"):
        raise ValueError("gripper preset must be open or close")
    profile.setdefault("gripper", {})[name] = float(rad)
    save_profile(path, profile)
    print(f"gripper {name} = {rad:.4f} rad")


def set_side_pick_lift_z(profile: dict, path: Path, z: float) -> None:
    profile["side_pick_lift_z"] = float(z)
    save_profile(path, profile)
    print(f"side_pick_lift_z = {z:.4f} m")


def pose_from_profile(profile: dict, group: str, name: str) -> tuple[float, float, float, float, float]:
    try:
        values = profile[group][name]
    except KeyError as exc:
        raise ValueError(f"unknown {group} pose: {name}") from exc
    if not isinstance(values, list) or len(values) != 5:
        raise ValueError(f"{group} pose {name} must be [x, y, z, roll, pitch]")
    return tuple(float(value) for value in values)  # type: ignore[return-value]


def top_pose_from_profile(profile: dict, name: str) -> tuple[float, float, float, float, float]:
    return pose_from_profile(profile, "top", name)


def side_pose_from_profile(profile: dict, name: str) -> tuple[float, float, float, float, float]:
    return pose_from_profile(profile, "side", name)


def side_pick_lift_z_from_profile(profile: dict) -> float:
    if "side_pick_lift_z" in profile:
        return float(profile["side_pick_lift_z"])
    return side_pose_from_profile(profile, "carry_safe")[2]


def side_pick_lift_pose(profile: dict) -> tuple[float, float, float, float, float]:
    insert = side_pose_from_profile(profile, "pick_insert")
    return (insert[0], insert[1], side_pick_lift_z_from_profile(profile), insert[3], insert[4])


def tolerance_from_profile(profile: dict, name: str, default: float) -> float:
    return float(profile.get("tolerance", {}).get(name, default))


def layer_z_from_profile(profile: dict, layer: int) -> float:
    try:
        return float(profile["layers"][str(layer)])
    except KeyError as exc:
        raise ValueError(f"unknown layer height: {layer}") from exc


def layer_z_for_direction(profile: dict, layer: int, direction: str) -> float:
    if direction not in ("up", "down"):
        return layer_z_from_profile(profile, layer)
    try:
        return float(profile[f"layers_{direction}"][str(layer)])
    except KeyError:
        return layer_z_from_profile(profile, layer)


def transfer_layer_direction(src_layer: int, dst_layer: int) -> str:
    if dst_layer > src_layer:
        return "up"
    if dst_layer < src_layer:
        return "down"
    return "flat"


def gripper_from_profile(profile: dict, name: str) -> float:
    try:
        return float(profile["gripper"][name])
    except KeyError as exc:
        raise ValueError(f"unknown gripper preset: {name}") from exc


def gripper_tolerance_from_profile(profile: dict, name: str) -> float:
    return float(profile.get("gripper", {}).get(f"{name}_tolerance", 0.20))


def replace_define(text: str, name: str, value: float) -> str:
    lines = text.splitlines()
    prefix = f"#define {name} "
    replacement = f"#define {name} ({value:.6f}f)"
    for index, line in enumerate(lines):
        if line.startswith(prefix):
            lines[index] = replacement
            return "\n".join(lines) + ("\n" if text.endswith("\n") else "")
    raise ValueError(f"define not found: {name}")


def profile_to_config_values(profile: dict) -> dict[str, float]:
    top_mapping: dict[str, tuple[str, int]] = {
        "DEMO_TOP_PICK_APPROACH_X_M": ("pick_approach", 0),
        "DEMO_TOP_PICK_APPROACH_Y_M": ("pick_approach", 1),
        "DEMO_TOP_PICK_APPROACH_Z_M": ("pick_approach", 2),
        "DEMO_TOP_PICK_APPROACH_ROLL_RAD": ("pick_approach", 3),
        "DEMO_TOP_PICK_APPROACH_PITCH_RAD": ("pick_approach", 4),
        "DEMO_TOP_PICK_DOWN_X_M": ("pick_down", 0),
        "DEMO_TOP_PICK_DOWN_Y_M": ("pick_down", 1),
        "DEMO_TOP_PICK_DOWN_Z_M": ("pick_down", 2),
        "DEMO_TOP_PICK_DOWN_ROLL_RAD": ("pick_down", 3),
        "DEMO_TOP_PICK_DOWN_PITCH_RAD": ("pick_down", 4),
        "DEMO_TOP_PICK_LIFT_X_M": ("pick_lift", 0),
        "DEMO_TOP_PICK_LIFT_Y_M": ("pick_lift", 1),
        "DEMO_TOP_PICK_LIFT_Z_M": ("pick_lift", 2),
        "DEMO_TOP_PICK_LIFT_ROLL_RAD": ("pick_lift", 3),
        "DEMO_TOP_PICK_LIFT_PITCH_RAD": ("pick_lift", 4),
        "DEMO_TOP_PLACE_APPROACH_X_M": ("place_approach", 0),
        "DEMO_TOP_PLACE_APPROACH_Y_M": ("place_approach", 1),
        "DEMO_TOP_PLACE_APPROACH_Z_M": ("place_approach", 2),
        "DEMO_TOP_PLACE_APPROACH_ROLL_RAD": ("place_approach", 3),
        "DEMO_TOP_PLACE_APPROACH_PITCH_RAD": ("place_approach", 4),
        "DEMO_TOP_PLACE_DOWN_X_M": ("place_down", 0),
        "DEMO_TOP_PLACE_DOWN_Y_M": ("place_down", 1),
        "DEMO_TOP_PLACE_DOWN_Z_M": ("place_down", 2),
        "DEMO_TOP_PLACE_DOWN_ROLL_RAD": ("place_down", 3),
        "DEMO_TOP_PLACE_DOWN_PITCH_RAD": ("place_down", 4),
        "DEMO_TOP_PLACE_LIFT_X_M": ("place_lift", 0),
        "DEMO_TOP_PLACE_LIFT_Y_M": ("place_lift", 1),
        "DEMO_TOP_PLACE_LIFT_Z_M": ("place_lift", 2),
        "DEMO_TOP_PLACE_LIFT_ROLL_RAD": ("place_lift", 3),
        "DEMO_TOP_PLACE_LIFT_PITCH_RAD": ("place_lift", 4),
    }
    side_mapping: dict[str, tuple[str, int]] = {
        "DEMO_SIDE_SAFE_X_M": ("safe", 0),
        "DEMO_SIDE_SAFE_Y_M": ("safe", 1),
        "DEMO_SIDE_SAFE_Z_M": ("safe", 2),
        "DEMO_SIDE_SAFE_ROLL_RAD": ("safe", 3),
        "DEMO_SIDE_SAFE_PITCH_RAD": ("safe", 4),
        "DEMO_SIDE_PICK_APPROACH_X_M": ("pick_approach", 0),
        "DEMO_SIDE_PICK_APPROACH_Y_M": ("pick_approach", 1),
        "DEMO_SIDE_PICK_APPROACH_Z_M": ("pick_approach", 2),
        "DEMO_SIDE_PICK_APPROACH_ROLL_RAD": ("pick_approach", 3),
        "DEMO_SIDE_PICK_APPROACH_PITCH_RAD": ("pick_approach", 4),
        "DEMO_SIDE_PICK_CLEARANCE_X_M": ("pick_clearance", 0),
        "DEMO_SIDE_PICK_CLEARANCE_Y_M": ("pick_clearance", 1),
        "DEMO_SIDE_PICK_CLEARANCE_Z_M": ("pick_clearance", 2),
        "DEMO_SIDE_PICK_CLEARANCE_ROLL_RAD": ("pick_clearance", 3),
        "DEMO_SIDE_PICK_CLEARANCE_PITCH_RAD": ("pick_clearance", 4),
        "DEMO_SIDE_PICK_PRE_INSERT_X_M": ("pick_pre_insert", 0),
        "DEMO_SIDE_PICK_PRE_INSERT_Y_M": ("pick_pre_insert", 1),
        "DEMO_SIDE_PICK_PRE_INSERT_Z_M": ("pick_pre_insert", 2),
        "DEMO_SIDE_PICK_PRE_INSERT_ROLL_RAD": ("pick_pre_insert", 3),
        "DEMO_SIDE_PICK_PRE_INSERT_PITCH_RAD": ("pick_pre_insert", 4),
        "DEMO_SIDE_PICK_INSERT_X_M": ("pick_insert", 0),
        "DEMO_SIDE_PICK_INSERT_Y_M": ("pick_insert", 1),
        "DEMO_SIDE_PICK_INSERT_Z_M": ("pick_insert", 2),
        "DEMO_SIDE_PICK_INSERT_ROLL_RAD": ("pick_insert", 3),
        "DEMO_SIDE_PICK_INSERT_PITCH_RAD": ("pick_insert", 4),
        "DEMO_SIDE_CARRY_SAFE_X_M": ("carry_safe", 0),
        "DEMO_SIDE_CARRY_SAFE_Y_M": ("carry_safe", 1),
        "DEMO_SIDE_CARRY_SAFE_Z_M": ("carry_safe", 2),
        "DEMO_SIDE_CARRY_SAFE_ROLL_RAD": ("carry_safe", 3),
        "DEMO_SIDE_CARRY_SAFE_PITCH_RAD": ("carry_safe", 4),
        "DEMO_SIDE_PLACE_APPROACH_X_M": ("place_approach", 0),
        "DEMO_SIDE_PLACE_APPROACH_Y_M": ("place_approach", 1),
        "DEMO_SIDE_PLACE_APPROACH_Z_M": ("place_approach", 2),
        "DEMO_SIDE_PLACE_APPROACH_ROLL_RAD": ("place_approach", 3),
        "DEMO_SIDE_PLACE_APPROACH_PITCH_RAD": ("place_approach", 4),
        "DEMO_SIDE_PLACE_CLEARANCE_X_M": ("place_clearance", 0),
        "DEMO_SIDE_PLACE_CLEARANCE_Y_M": ("place_clearance", 1),
        "DEMO_SIDE_PLACE_CLEARANCE_Z_M": ("place_clearance", 2),
        "DEMO_SIDE_PLACE_CLEARANCE_ROLL_RAD": ("place_clearance", 3),
        "DEMO_SIDE_PLACE_CLEARANCE_PITCH_RAD": ("place_clearance", 4),
        "DEMO_SIDE_PLACE_PRE_INSERT_X_M": ("place_pre_insert", 0),
        "DEMO_SIDE_PLACE_PRE_INSERT_Y_M": ("place_pre_insert", 1),
        "DEMO_SIDE_PLACE_PRE_INSERT_Z_M": ("place_pre_insert", 2),
        "DEMO_SIDE_PLACE_PRE_INSERT_ROLL_RAD": ("place_pre_insert", 3),
        "DEMO_SIDE_PLACE_PRE_INSERT_PITCH_RAD": ("place_pre_insert", 4),
        "DEMO_SIDE_PLACE_INSERT_X_M": ("place_insert", 0),
        "DEMO_SIDE_PLACE_INSERT_Y_M": ("place_insert", 1),
        "DEMO_SIDE_PLACE_INSERT_Z_M": ("place_insert", 2),
        "DEMO_SIDE_PLACE_INSERT_ROLL_RAD": ("place_insert", 3),
        "DEMO_SIDE_PLACE_INSERT_PITCH_RAD": ("place_insert", 4),
        "DEMO_SIDE_RETREAT_SAFE_X_M": ("retreat_safe", 0),
        "DEMO_SIDE_RETREAT_SAFE_Y_M": ("retreat_safe", 1),
        "DEMO_SIDE_RETREAT_SAFE_Z_M": ("retreat_safe", 2),
        "DEMO_SIDE_RETREAT_SAFE_ROLL_RAD": ("retreat_safe", 3),
        "DEMO_SIDE_RETREAT_SAFE_PITCH_RAD": ("retreat_safe", 4),
    }
    values = {
        "DEMO_LAYER_1_Z_M": layer_z_from_profile(profile, 1),
        "DEMO_LAYER_2_Z_M": layer_z_from_profile(profile, 2),
        "DEMO_LAYER_3_Z_M": layer_z_from_profile(profile, 3),
        "DEMO_GRIPPER_OPEN_RAD": gripper_from_profile(profile, "open"),
        "DEMO_GRIPPER_CLOSE_RAD": gripper_from_profile(profile, "close"),
        "DEMO_SIDE_PICK_LIFT_Z_M": side_pick_lift_z_from_profile(profile),
    }
    for define_name, (pose_name, pose_index) in top_mapping.items():
        values[define_name] = top_pose_from_profile(profile, pose_name)[pose_index]
    for define_name, (pose_name, pose_index) in side_mapping.items():
        values[define_name] = side_pose_from_profile(profile, pose_name)[pose_index]
    return values


def write_profile_to_config(profile: dict, config_path: Path = CONFIG_PATH) -> None:
    text = config_path.read_text(encoding="utf-8")
    for name, value in profile_to_config_values(profile).items():
        text = replace_define(text, name, value)
    config_path.write_text(text, encoding="utf-8")
    print(f"updated {config_path}")


@dataclass
class DecodedFrame:
    src: int
    target: int
    cmd: int
    seq: int
    payload: bytes


@dataclass
class CommandResult:
    target: int
    request_seq: int
    request_cmd: int
    request_source: int
    result: int
    reject_reason: int
    fault_reason: int
    state_after: int
    diag_code: int
    current: tuple[float, ...]
    requested: tuple[float, ...]
    accepted: tuple[float, ...]


@dataclass
class ArmStatus:
    tick_ms: int
    state: int
    status: int
    is_busy: int
    has_fault: int
    active_cmd: int
    active_source: int
    active_seq: int
    diag_code: int
    current_x: float
    current_y: float
    current_z: float
    target_x: float
    target_y: float
    target_z: float
    current_gripper_rad: float
    target_gripper_rad: float
    fault_flags: int


@dataclass
class LiftStatus:
    tick_ms: int
    state: int
    home_state: int
    is_homed: int
    is_busy: int
    has_fault: int
    fault_reason: int
    motor_blocked: int
    home_sensor_level: int
    current_z: float
    target_z: float
    current_v: float
    target_v: float
    position_error: float
    motor_total_encoder_count: int


@dataclass
class DemoStatus:
    tick_ms: int
    state: int
    fault: int
    demo_id: int
    src_layer: int
    dst_layer: int
    variant: int
    step_index: int
    step_count: int
    active: int


RESULT_NAMES = {
    RESULT_NONE: "NONE",
    RESULT_ACCEPTED: "ACCEPTED",
    RESULT_REJECTED: "REJECTED",
    RESULT_COMPLETED: "COMPLETED",
    RESULT_ABORTED: "ABORTED",
    RESULT_FAILED: "FAILED",
    RESULT_SUPERSEDED: "SUPERSEDED",
}


def result_name(value: int) -> str:
    return RESULT_NAMES.get(value, f"UNKNOWN({value})")


def decode_demo_result(payload: bytes) -> CommandResult | None:
    fmt = "<IBBBBBBBBBBBBBBBB"
    if len(payload) < struct.calcsize(fmt):
        return None
    values = struct.unpack_from(fmt, payload)
    return CommandResult(
        target=CTRL_TARGET_SYSTEM,
        request_seq=values[1],
        request_cmd=values[2],
        request_source=values[3],
        result=values[4],
        reject_reason=values[5],
        fault_reason=values[6],
        state_after=values[7],
        diag_code=0,
        requested=tuple(float(value) for value in values[8:12]),
        accepted=(),
        current=tuple(float(value) for value in values[12:14]),
    )


def decode_demo_status(payload: bytes) -> DemoStatus | None:
    fmt = "<IBBBBBBBBBBBB"
    if len(payload) < struct.calcsize(fmt):
        return None
    values = struct.unpack_from(fmt, payload)
    return DemoStatus(
        tick_ms=values[0],
        state=values[1],
        fault=values[2],
        demo_id=values[3],
        src_layer=values[4],
        dst_layer=values[5],
        variant=values[6],
        step_index=values[7],
        step_count=values[8],
        active=values[9],
    )


def decode_arm_result(payload: bytes) -> CommandResult | None:
    fmt = "<IBBBBBBBBffffffffffff"
    if len(payload) < struct.calcsize(fmt):
        return None
    values = struct.unpack_from(fmt, payload)
    return CommandResult(
        target=CTRL_TARGET_ARM,
        request_seq=values[1],
        request_cmd=values[2],
        request_source=values[3],
        result=values[4],
        reject_reason=values[5],
        fault_reason=values[6],
        state_after=values[7],
        diag_code=values[8],
        requested=tuple(float(value) for value in values[9:13]),
        accepted=tuple(float(value) for value in values[13:17]),
        current=tuple(float(value) for value in values[17:21]),
    )


def decode_lift_result(payload: bytes) -> CommandResult | None:
    fmt = "<IBBBBBBBBfffff"
    if len(payload) < struct.calcsize(fmt):
        return None
    values = struct.unpack_from(fmt, payload)
    return CommandResult(
        target=CTRL_TARGET_LIFT,
        request_seq=values[1],
        request_cmd=values[2],
        request_source=values[3],
        result=values[4],
        reject_reason=values[5],
        fault_reason=values[6],
        state_after=values[7],
        diag_code=0,
        requested=(float(values[9]),),
        accepted=(float(values[10]),),
        current=(float(values[11]),),
    )


def decode_arm_status(payload: bytes) -> ArmStatus | None:
    fmt = "<IBBBBBBBBffffffffI"
    if len(payload) < struct.calcsize(fmt):
        return None
    return ArmStatus(*struct.unpack_from(fmt, payload))


def decode_lift_status(payload: bytes) -> LiftStatus | None:
    fmt = "<IBBBBBBBBfffffi"
    if len(payload) < struct.calcsize(fmt):
        return None
    return LiftStatus(*struct.unpack_from(fmt, payload))


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[DecodedFrame]:
        self.buffer.extend(data)
        frames: list[DecodedFrame] = []

        while True:
            sof_index = self.buffer.find(SOF)
            if sof_index < 0:
                self.buffer.clear()
                return frames
            if sof_index > 0:
                del self.buffer[:sof_index]
            if len(self.buffer) < 8:
                return frames

            src, target, cmd, seq, length = self.buffer[2:7]
            frame_len = 2 + 5 + length + 1
            if length > CTRL_PROTOCOL_MAX_PAYLOAD:
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_len:
                return frames

            raw = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]
            body = raw[2:-1]
            if crc8_atm(body) != raw[-1]:
                continue
            frames.append(DecodedFrame(src, target, cmd, seq, raw[7:-1]))


class DemoConsole:
    def __init__(
        self,
        port: str,
        baudrate: int,
        source: int,
        dry_run: bool = False,
        timeout: float = 0.05,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.source = source
        self.dry_run = dry_run
        self.timeout = timeout
        self.seq = 0
        self.ser = None
        self.parser = FrameParser()
        self.latest_arm: ArmStatus | None = None
        self.latest_lift: LiftStatus | None = None
        self.latest_demo: DemoStatus | None = None

    def __enter__(self) -> "DemoConsole":
        if not self.dry_run:
            if serial is None:
                raise RuntimeError("pyserial is not installed. Run: python -m pip install pyserial")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False,
            )
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.2)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.ser is not None:
            self.ser.close()

    def next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def send(self, target: int, cmd: int, payload: bytes = b"") -> int:
        seq = self.next_seq()
        frame = encode_frame(self.source, target, cmd, seq, payload)
        print(f"TX {hex_bytes(frame)}")
        if self.ser is not None:
            self.ser.write(frame)
            self.ser.flush()
        return seq

    def send_confirmed(self, target: int, cmd: int, payload: bytes = b"", timeout_s: float = 15.0) -> CommandResult:
        seq = self.send(target, cmd, payload)
        return self.wait_result(target, seq, timeout_s)

    def is_gripper_result_close(self, result: CommandResult, rad: float, tolerance: float) -> bool:
        if not result.current:
            return False
        return abs(result.current[-1] - rad) <= tolerance

    def is_arm_result_z_close(self, result: CommandResult, target_z: float, tolerance: float) -> bool:
        if len(result.current) < 3:
            return False
        return abs(result.current[2] - target_z) <= tolerance

    def is_latest_gripper_close(self, rad: float, tolerance: float) -> bool:
        arm = self.latest_arm
        if arm is None or arm.has_fault:
            return False
        return abs(arm.current_gripper_rad - rad) <= tolerance

    def is_latest_arm_z_close(self, target_z: float, tolerance: float) -> bool:
        arm = self.latest_arm
        if arm is None or arm.has_fault:
            return False
        return abs(arm.current_z - target_z) <= tolerance

    def send_gripper_confirmed(self, rad: float, tolerance: float = 0.12) -> CommandResult:
        seq = self.send(CTRL_TARGET_ARM, CTRL_ARM_CMD_GRIPPER, struct.pack("<f", rad))
        if self.ser is None:
            print(f"dry-run wait target=0x{CTRL_TARGET_ARM:02X} seq={seq}")
            return CommandResult(
                CTRL_TARGET_ARM,
                seq,
                CTRL_ARM_CMD_GRIPPER,
                self.source,
                RESULT_COMPLETED,
                0,
                0,
                0,
                0,
                (),
                (),
                (),
            )

        deadline = time.monotonic() + 5.0
        accepted = False
        while time.monotonic() < deadline:
            for result in self.poll_frames(verbose=False):
                if result.target != CTRL_TARGET_ARM or result.request_seq != seq:
                    continue
                print(
                    f"RX result target=0x{CTRL_TARGET_ARM:02X} seq={seq} "
                    f"cmd=0x{result.request_cmd:02X} {result_name(result.result)} "
                    f"reject={result.reject_reason} fault={result.fault_reason} "
                    f"state={result.state_after} diag={result.diag_code} "
                    f"current={tuple(round(value, 4) for value in result.current)}"
                )
                if result.result == RESULT_ACCEPTED:
                    accepted = True
                    continue
                if result.result == RESULT_COMPLETED:
                    return result
                if self.is_gripper_result_close(result, rad, tolerance):
                    current = result.current[-1]
                    error = abs(current - rad)
                    print(
                        f"WARNING gripper accepted by tolerance: target={rad:.4f} "
                        f"current={current:.4f} error={error:.4f} tolerance={tolerance:.4f}"
                    )
                    return result
                raise RuntimeError(
                    f"command failed target=0x{CTRL_TARGET_ARM:02X} seq={seq} "
                    f"result={result_name(result.result)} reject={result.reject_reason} "
                    f"fault={result.fault_reason} state={result.state_after} "
                    f"diag={result.diag_code} current={result.current}"
                )

            if accepted and self.is_latest_gripper_close(rad, tolerance):
                arm = self.latest_arm
                if arm is not None:
                    error = abs(arm.current_gripper_rad - rad)
                    print(
                        f"WARNING gripper accepted by status tolerance: target={rad:.4f} "
                        f"current={arm.current_gripper_rad:.4f} error={error:.4f} "
                        f"tolerance={tolerance:.4f}"
                    )
                    return CommandResult(
                        CTRL_TARGET_ARM,
                        seq,
                        CTRL_ARM_CMD_GRIPPER,
                        self.source,
                        RESULT_COMPLETED,
                        0,
                        0,
                        arm.state,
                        arm.diag_code,
                        (arm.current_x, arm.current_y, arm.current_z, arm.current_gripper_rad),
                        (rad,),
                        (rad,),
                    )

        raise TimeoutError(f"timed out waiting for gripper target={rad:.4f} tolerance={tolerance:.4f}")

    def send_arm_pose_z_tolerant(
        self,
        label: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        z_tolerance: float,
        timeout_s: float = 5.0,
    ) -> CommandResult:
        seq = self.send(CTRL_TARGET_ARM, CTRL_ARM_CMD_MOVE_POSE, struct.pack("<fffff", x, y, z, roll, pitch))
        if self.ser is None:
            print(f"dry-run wait target=0x{CTRL_TARGET_ARM:02X} seq={seq}")
            return CommandResult(
                CTRL_TARGET_ARM,
                seq,
                CTRL_ARM_CMD_MOVE_POSE,
                self.source,
                RESULT_COMPLETED,
                0,
                0,
                0,
                0,
                (),
                (),
                (),
            )

        deadline = time.monotonic() + timeout_s
        accepted = False
        while time.monotonic() < deadline:
            for result in self.poll_frames(verbose=False):
                if result.target != CTRL_TARGET_ARM or result.request_seq != seq:
                    continue
                print(
                    f"RX result target=0x{CTRL_TARGET_ARM:02X} seq={seq} "
                    f"cmd=0x{result.request_cmd:02X} {result_name(result.result)} "
                    f"reject={result.reject_reason} fault={result.fault_reason} "
                    f"state={result.state_after} diag={result.diag_code} "
                    f"current={tuple(round(value, 4) for value in result.current)}"
                )
                if result.result == RESULT_ACCEPTED:
                    accepted = True
                    continue
                if result.result == RESULT_COMPLETED:
                    return result
                if self.is_arm_result_z_close(result, z, z_tolerance):
                    current_z = result.current[2]
                    error = abs(current_z - z)
                    print(
                        f"WARNING {label} accepted by z tolerance: target_z={z:.4f} "
                        f"current_z={current_z:.4f} error={error:.4f} tolerance={z_tolerance:.4f}"
                    )
                    return result
                raise RuntimeError(
                    f"command failed target=0x{CTRL_TARGET_ARM:02X} seq={seq} "
                    f"result={result_name(result.result)} reject={result.reject_reason} "
                    f"fault={result.fault_reason} state={result.state_after} "
                    f"diag={result.diag_code} current={result.current}"
                )

            if accepted and self.is_latest_arm_z_close(z, z_tolerance):
                arm = self.latest_arm
                if arm is not None:
                    error = abs(arm.current_z - z)
                    print(
                        f"WARNING {label} accepted by status z tolerance: target_z={z:.4f} "
                        f"current_z={arm.current_z:.4f} error={error:.4f} tolerance={z_tolerance:.4f}"
                    )
                    return CommandResult(
                        CTRL_TARGET_ARM,
                        seq,
                        CTRL_ARM_CMD_MOVE_POSE,
                        self.source,
                        RESULT_COMPLETED,
                        0,
                        0,
                        arm.state,
                        arm.diag_code,
                        (arm.current_x, arm.current_y, arm.current_z, arm.current_gripper_rad),
                        (x, y, z, roll, pitch),
                        (x, y, z, roll, pitch),
                    )

        raise TimeoutError(f"timed out waiting for {label} target_z={z:.4f} tolerance={z_tolerance:.4f}")

    def handle_frame(self, frame: DecodedFrame, verbose: bool = False) -> CommandResult | None:
        result = None
        if frame.target == CTRL_TARGET_SYSTEM and frame.cmd == CTRL_SYS_RPT_DEMO_STATUS:
            self.latest_demo = decode_demo_status(frame.payload)
        elif frame.target == CTRL_TARGET_SYSTEM and frame.cmd == CTRL_SYS_RPT_DEMO_RESULT:
            result = decode_demo_result(frame.payload)
        elif frame.target == CTRL_TARGET_ARM and frame.cmd == CTRL_ARM_RPT_STATUS:
            self.latest_arm = decode_arm_status(frame.payload)
        elif frame.target == CTRL_TARGET_LIFT and frame.cmd == CTRL_LIFT_RPT_STATUS:
            self.latest_lift = decode_lift_status(frame.payload)
        elif frame.target == CTRL_TARGET_ARM and frame.cmd == CTRL_ARM_RPT_RESULT:
            result = decode_arm_result(frame.payload)
        elif frame.target == CTRL_TARGET_LIFT and frame.cmd == CTRL_LIFT_RPT_RESULT:
            result = decode_lift_result(frame.payload)

        if verbose:
            print(
                "RX "
                f"src=0x{frame.src:02X} target=0x{frame.target:02X} "
                f"cmd=0x{frame.cmd:02X} seq={frame.seq} len={len(frame.payload)} "
                f"payload={hex_bytes(frame.payload)}"
            )
            if result is not None:
                print(
                    f"   result req_seq={result.request_seq} "
                    f"req_cmd=0x{result.request_cmd:02X} {result_name(result.result)} "
                    f"reject={result.reject_reason} fault={result.fault_reason} "
                    f"state={result.state_after} diag={result.diag_code} "
                    f"current={tuple(round(value, 4) for value in result.current)}"
                )
            if self.latest_demo is not None and frame.target == CTRL_TARGET_SYSTEM:
                demo = self.latest_demo
                print(
                    f"   demo state={demo.state} fault={demo.fault} id={demo.demo_id} "
                    f"src={demo.src_layer} dst={demo.dst_layer} variant={demo.variant} "
                    f"step={demo.step_index}/{demo.step_count} active={demo.active}"
                )
        return result

    def poll_frames(self, verbose: bool = False) -> list[CommandResult]:
        if self.ser is None:
            return []
        results: list[CommandResult] = []
        data = self.ser.read(256)
        for frame in self.parser.feed(data):
            result = self.handle_frame(frame, verbose=verbose)
            if result is not None:
                results.append(result)
        return results

    def wait_result(self, target: int, seq: int, timeout_s: float, accept_is_final: bool = False) -> CommandResult:
        if self.ser is None:
            print(f"dry-run wait target=0x{target:02X} seq={seq}")
            return CommandResult(target, seq, 0, self.source, RESULT_COMPLETED, 0, 0, 0, 0, (), (), ())

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for result in self.poll_frames(verbose=False):
                if result.target != target or result.request_seq != seq:
                    continue
                print(
                    f"RX result target=0x{target:02X} seq={seq} "
                    f"cmd=0x{result.request_cmd:02X} {result_name(result.result)} "
                    f"reject={result.reject_reason} fault={result.fault_reason} "
                    f"state={result.state_after} diag={result.diag_code} "
                    f"current={tuple(round(value, 4) for value in result.current)}"
                )
                if result.result == RESULT_ACCEPTED:
                    if accept_is_final:
                        return result
                    continue
                if result.result == RESULT_COMPLETED:
                    return result
                raise RuntimeError(
                    f"command failed target=0x{target:02X} seq={seq} "
                    f"result={result_name(result.result)} reject={result.reject_reason} "
                    f"fault={result.fault_reason} state={result.state_after} "
                    f"diag={result.diag_code} current={result.current}"
                )

        detail = ""
        if target == CTRL_TARGET_ARM and self.latest_arm is not None:
            arm = self.latest_arm
            detail = (
                f" arm current=({arm.current_x:.4f},{arm.current_y:.4f},{arm.current_z:.4f})"
                f" gripper={arm.current_gripper_rad:.4f} busy={arm.is_busy}"
                f" fault={arm.has_fault} diag={arm.diag_code}"
            )
        elif target == CTRL_TARGET_LIFT and self.latest_lift is not None:
            lift = self.latest_lift
            detail = (
                f" lift z={lift.current_z:.4f} target={lift.target_z:.4f}"
                f" homed={lift.is_homed} busy={lift.is_busy}"
                f" fault={lift.has_fault} reason={lift.fault_reason}"
            )
        raise TimeoutError(f"timed out waiting for target=0x{target:02X} seq={seq}{detail}")

    def demo_run(self, demo_id: int, src_layer: int = 0, dst_layer: int = 0, variant: int = 0) -> None:
        payload = struct.pack("<BBBB", demo_id, src_layer, dst_layer, variant)
        seq = self.send(CTRL_TARGET_SYSTEM, CTRL_SYS_CMD_DEMO_RUN, payload)
        self.wait_result(CTRL_TARGET_SYSTEM, seq, 2.0, accept_is_final=True)

    def demo_stop(self) -> None:
        seq = self.send(CTRL_TARGET_SYSTEM, CTRL_SYS_CMD_DEMO_STOP)
        self.wait_result(CTRL_TARGET_SYSTEM, seq, 2.0, accept_is_final=True)

    def demo_home(self) -> None:
        seq = self.send(CTRL_TARGET_SYSTEM, CTRL_SYS_CMD_DEMO_HOME)
        self.wait_result(CTRL_TARGET_SYSTEM, seq, 2.0, accept_is_final=True)

    def chassis_stop(self) -> None:
        self.send(CTRL_TARGET_CHASSIS, CTRL_CHS_CMD_STOP)

    def arm_stop(self) -> None:
        self.send(CTRL_TARGET_ARM, CTRL_ARM_CMD_STOP)

    def arm_home(self, confirmed: bool = False) -> None:
        if confirmed:
            self.send_confirmed(CTRL_TARGET_ARM, CTRL_ARM_CMD_HOME, timeout_s=20.0)
        else:
            self.send(CTRL_TARGET_ARM, CTRL_ARM_CMD_HOME)

    def arm_xyz(self, x: float, y: float, z: float, confirmed: bool = False) -> None:
        payload = struct.pack("<fff", x, y, z)
        if confirmed:
            self.send_confirmed(CTRL_TARGET_ARM, CTRL_ARM_CMD_MOVE_XYZ, payload, timeout_s=15.0)
        else:
            self.send(CTRL_TARGET_ARM, CTRL_ARM_CMD_MOVE_XYZ, payload)

    def arm_pose(self, x: float, y: float, z: float, roll: float, pitch: float, confirmed: bool = False) -> None:
        payload = struct.pack("<fffff", x, y, z, roll, pitch)
        if confirmed:
            self.send_confirmed(CTRL_TARGET_ARM, CTRL_ARM_CMD_MOVE_POSE, payload, timeout_s=15.0)
        else:
            self.send(CTRL_TARGET_ARM, CTRL_ARM_CMD_MOVE_POSE, payload)

    def gripper(self, rad: float, confirmed: bool = False) -> None:
        if confirmed:
            self.send_gripper_confirmed(rad)
        else:
            self.send(CTRL_TARGET_ARM, CTRL_ARM_CMD_GRIPPER, struct.pack("<f", rad))

    def gripper_assume_ok(self, rad: float) -> None:
        try:
            self.gripper(rad, confirmed=True)
        except Exception as exc:
            print(f"WARNING gripper assumed OK after error: {exc}")

    def gripper_send_only(self, rad: float) -> None:
        self.gripper(rad, confirmed=False)

    def gripper_close_wait_ok(self, profile: dict) -> None:
        self.send_gripper_confirmed(
            gripper_from_profile(profile, "close"),
            tolerance=gripper_tolerance_from_profile(profile, "close"),
        )

    def arm_pose_assume_ok(
        self,
        label: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
    ) -> None:
        try:
            self.arm_pose(x, y, z, roll, pitch, confirmed=True)
        except Exception as exc:
            print(f"WARNING {label} assumed OK after error: {exc}")

    def arm_pose_send_only(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
    ) -> None:
        self.arm_pose(x, y, z, roll, pitch, confirmed=False)

    def lift_stop(self) -> None:
        self.send(CTRL_TARGET_LIFT, CTRL_LIFT_CMD_STOP)

    def lift_home(self, confirmed: bool = False) -> None:
        if confirmed:
            self.send_confirmed(CTRL_TARGET_LIFT, CTRL_LIFT_CMD_HOME, timeout_s=20.0)
        else:
            self.send(CTRL_TARGET_LIFT, CTRL_LIFT_CMD_HOME)

    def lift_z(self, z: float, confirmed: bool = False) -> None:
        payload = struct.pack("<f", z)
        if confirmed:
            self.send_confirmed(CTRL_TARGET_LIFT, CTRL_LIFT_CMD_MOVE_Z, payload, timeout_s=20.0)
        else:
            self.send(CTRL_TARGET_LIFT, CTRL_LIFT_CMD_MOVE_Z, payload)

    def lift_layer(self, profile: dict, layer: int, confirmed: bool = False) -> None:
        self.lift_z(layer_z_from_profile(profile, layer), confirmed=confirmed)

    def lift_layer_direction(self, profile: dict, layer: int, direction: str, confirmed: bool = False) -> None:
        self.lift_z(layer_z_for_direction(profile, layer, direction), confirmed=confirmed)

    def lift_layer_assume_ok(self, profile: dict, layer: int, label: str) -> None:
        try:
            self.lift_layer(profile, layer, confirmed=True)
        except Exception as exc:
            print(f"WARNING {label} assumed OK after error: {exc}")

    def run_static_top_sequence(self, profile: dict, step_delay: float, confirmed: bool = True) -> None:
        self.chassis_stop()
        time.sleep(step_delay)
        self.poll_frames()
        if self.latest_lift is None or not self.latest_lift.is_homed:
            self.lift_home(confirmed=confirmed)
            time.sleep(step_delay)
        else:
            print("LIFT_HOME... skipped, already homed")
        self.lift_z(float(profile.get("static_lift_z_m", 0.168)), confirmed=confirmed)
        time.sleep(step_delay)
        self.send_gripper_confirmed(
            gripper_from_profile(profile, "open"),
            tolerance=gripper_tolerance_from_profile(profile, "open"),
        )
        time.sleep(step_delay)
        self.arm_pose(*top_pose_from_profile(profile, "safe"), confirmed=confirmed)
        time.sleep(step_delay)
        self.send_arm_pose_z_tolerant(
            "top pick hover",
            *top_pose_from_profile(profile, "pick_hover"),
            z_tolerance=tolerance_from_profile(profile, "top_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_pose(*top_pose_from_profile(profile, "pick_down"), confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper_close_wait_ok(profile)
        time.sleep(step_delay)
        self.send_arm_pose_z_tolerant(
            "top pick lift",
            *top_pose_from_profile(profile, "pick_lift"),
            z_tolerance=tolerance_from_profile(profile, "top_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_pose(*top_pose_from_profile(profile, "safe"), confirmed=confirmed)
        time.sleep(step_delay)
        move_deadline = time.monotonic() + 1.8
        while time.monotonic() < move_deadline:
            self.send(CTRL_TARGET_CHASSIS, CTRL_CHS_CMD_MOV,
                      struct.pack("<Bfff", 0, math.pi / 2.0, 0.20, 0.0))
            time.sleep(0.2)
        self.chassis_stop()
        time.sleep(0.5)
        self.send_arm_pose_z_tolerant(
            "top place approach",
            *top_pose_from_profile(profile, "place_approach"),
            z_tolerance=tolerance_from_profile(profile, "top_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_pose(*top_pose_from_profile(profile, "place_down"), confirmed=confirmed)
        time.sleep(step_delay)
        self.send_gripper_confirmed(
            gripper_from_profile(profile, "open"),
            tolerance=gripper_tolerance_from_profile(profile, "open"),
        )
        time.sleep(step_delay)
        self.send_arm_pose_z_tolerant(
            "top place lift",
            *top_pose_from_profile(profile, "place_lift"),
            z_tolerance=tolerance_from_profile(profile, "top_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_home(confirmed=confirmed)

    def run_side_transfer_sequence(
        self,
        profile: dict,
        src_layer: int,
        dst_layer: int,
        step_delay: float,
        confirmed: bool = False,
    ) -> None:
        confirmed = True
        lift_direction = transfer_layer_direction(src_layer, dst_layer)
        self.chassis_stop()
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "safe"), confirmed=confirmed)
        time.sleep(step_delay)
        self.lift_layer_direction(profile, src_layer, lift_direction, confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper(gripper_from_profile(profile, "open"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "pick_approach"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "pick_clearance"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "pick_insert"), confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper_close_wait_ok(profile)
        time.sleep(step_delay)
        self.send_arm_pose_z_tolerant(
            "side pick lift",
            *side_pose_from_profile(profile, "pick_pre_insert"),
            z_tolerance=tolerance_from_profile(profile, "pick_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "carry_safe"), confirmed=confirmed)
        time.sleep(step_delay)
        self.lift_layer_direction(profile, dst_layer, lift_direction, confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "place_clearance"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "place_pre_insert"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "place_insert"), confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper(gripper_from_profile(profile, "open"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "place_approach"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "retreat_safe"), confirmed=confirmed)

    def run_side_pick_sequence(self, profile: dict, layer: int, step_delay: float, confirmed: bool = False) -> None:
        confirmed = True
        self.chassis_stop()
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "safe"), confirmed=confirmed)
        time.sleep(step_delay)
        self.lift_layer(profile, layer, confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper(gripper_from_profile(profile, "open"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "pick_approach"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "pick_clearance"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "pick_insert"), confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper_close_wait_ok(profile)
        time.sleep(step_delay)
        self.send_arm_pose_z_tolerant(
            "side pick lift",
            *side_pose_from_profile(profile, "pick_pre_insert"),
            z_tolerance=tolerance_from_profile(profile, "pick_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_pose(*side_pose_from_profile(profile, "carry_safe"), confirmed=confirmed)

    def run_cabinet_pull_sequence(self, profile: dict, step_delay: float, confirmed: bool = True) -> None:
        """Top-pose cabinet clamp + chassis backward pull to extract cabinet.

        Reuses existing top poses: safe, pick_approach, pick_down, pick_lift.
        Lift height and pull parameters are read from the profile.
        """
        cabinet_lift_z = float(profile.get("cabinet_lift_z_m", 0.168))
        pull_speed_mps = float(profile.get("cabinet_pull_speed_mps", 0.20))
        pull_duration_s = float(profile.get("cabinet_pull_duration_s", 1.0))

        self.chassis_stop()
        time.sleep(step_delay)
        self.poll_frames()
        if self.latest_lift is None or not self.latest_lift.is_homed:
            self.lift_home(confirmed=confirmed)
            time.sleep(step_delay)
        else:
            print("LIFT_HOME... skipped, already homed")
        self.lift_z(cabinet_lift_z, confirmed=confirmed)
        time.sleep(step_delay)
        self.send_gripper_confirmed(
            gripper_from_profile(profile, "open"),
            tolerance=gripper_tolerance_from_profile(profile, "open"),
        )
        time.sleep(step_delay)
        self.arm_pose(*top_pose_from_profile(profile, "safe"), confirmed=confirmed)
        time.sleep(step_delay)
        self.send_arm_pose_z_tolerant(
            "cabinet approach",
            *top_pose_from_profile(profile, "pick_approach"),
            z_tolerance=tolerance_from_profile(profile, "top_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_pose(*top_pose_from_profile(profile, "pick_down"), confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper_close_wait_ok(profile)
        time.sleep(step_delay)
        # Chassis backward pull: direction = pi (180 deg)
        pull_deadline = time.monotonic() + pull_duration_s
        while time.monotonic() < pull_deadline:
            self.send(CTRL_TARGET_CHASSIS, CTRL_CHS_CMD_MOV,
                      struct.pack("<Bfff", 0, math.pi, pull_speed_mps, 0.0))
            time.sleep(0.2)
        self.chassis_stop()
        time.sleep(0.5)
        # Release gripper and retreat
        self.send_gripper_confirmed(
            gripper_from_profile(profile, "open"),
            tolerance=gripper_tolerance_from_profile(profile, "open"),
        )
        time.sleep(step_delay)
        self.send_arm_pose_z_tolerant(
            "cabinet retreat",
            *top_pose_from_profile(profile, "pick_lift"),
            z_tolerance=tolerance_from_profile(profile, "top_lift_z", 0.05),
        )
        time.sleep(step_delay)
        self.arm_pose(*top_pose_from_profile(profile, "safe"), confirmed=confirmed)
        time.sleep(step_delay)
        self.arm_home(confirmed=confirmed)

    def run_key_turn_sequence(self, profile: dict, step_delay: float, confirmed: bool = True) -> None:
        """Direct key grip (roll=0) + rotate 90 deg (roll=pi/2) + chassis backward pull.

        Uses lift for height, goes straight to pick_insert without arm approach steps.
        After gripping, rotates roll 0→pi/2, chassis pulls back, release in place.
        """
        key_lift_z = float(profile.get("key_lift_z_m", 0.168))
        pull_speed_mps = float(profile.get("key_pull_speed_mps", 0.20))
        pull_duration_s = float(profile.get("key_pull_duration_s", 1.0))
        pull_omega = float(profile.get("key_pull_omega_radps", 0.5))

        self.chassis_stop()
        time.sleep(step_delay)
        self.poll_frames()
        if self.latest_lift is None or not self.latest_lift.is_homed:
            self.lift_home(confirmed=confirmed)
            time.sleep(step_delay)
        else:
            print("LIFT_HOME... skipped, already homed")
        self.lift_z(key_lift_z, confirmed=confirmed)
        time.sleep(step_delay)
        self.send_gripper_confirmed(
            gripper_from_profile(profile, "open"),
            tolerance=gripper_tolerance_from_profile(profile, "open"),
        )
        time.sleep(step_delay)
        # Directly to grip position (roll=0)
        x, y, z, _r, pitch = side_pose_from_profile(profile, "pick_insert")
        self.arm_pose(x, y, z, 0.0, pitch, confirmed=confirmed)
        time.sleep(step_delay)
        self.gripper_close_wait_ok(profile)
        time.sleep(step_delay)
        # Rotate key 90 deg: roll 0 → pi/2
        self.arm_pose(x, y, z, -math.pi / 2.0, pitch, confirmed=confirmed)
        time.sleep(step_delay)
        # Chassis backward pull with arc
        pull_deadline = time.monotonic() + pull_duration_s
        while time.monotonic() < pull_deadline:
            self.send(CTRL_TARGET_CHASSIS, CTRL_CHS_CMD_MOV,
                      struct.pack("<Bfff", 0, -3.0 * math.pi / 4.0, pull_speed_mps, pull_omega))
            time.sleep(0.2)
        self.chassis_stop()
        time.sleep(0.5)
        # Release in place
        self.send_gripper_confirmed(
            gripper_from_profile(profile, "open"),
            tolerance=gripper_tolerance_from_profile(profile, "open"),
        )
        time.sleep(step_delay)
        self.arm_home(confirmed=confirmed)

    def read_frames(self, seconds: float) -> None:
        if self.ser is None:
            print("monitor skipped in dry-run mode")
            return
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.poll_frames(verbose=True)


def positive_layer(value: str) -> int:
    layer = int(value)
    if layer not in (1, 2, 3):
        raise argparse.ArgumentTypeError("layer must be 1, 2, or 3")
    return layer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bluetooth demo console for MCU macro tuning")
    parser.add_argument("--port", default="COM13", help="serial port, default: COM13")
    parser.add_argument("--baudrate", type=int, default=115200, help="serial baudrate, default: 115200")
    parser.add_argument(
        "--source",
        choices=("bt", "host"),
        default="bt",
        help="declared protocol source; use bt for COM13 Bluetooth",
    )
    parser.add_argument("--dry-run", action="store_true", help="print frames without opening the port")
    parser.add_argument("--confirmed", action="store_true", help="wait for ARM/LIFT result completion after commands")
    parser.add_argument("--monitor", type=float, default=0.0, help="read telemetry for N seconds after command")
    parser.add_argument(
        "--profile",
        type=Path,
        default=DEFAULT_PROFILE_PATH,
        help=f"tuning profile JSON, default: {DEFAULT_PROFILE_PATH}",
    )
    parser.add_argument("--step-delay", type=float, default=0.0, help="delay between PC-side sequence commands")

    sub = parser.add_subparsers(dest="command", required=True)

    demo = sub.add_parser("demo", help="run MCU demo macro")
    demo_sub = demo.add_subparsers(dest="demo_command", required=True)
    demo_sub.add_parser("static", help="single-plane top-pose pick and place")
    pick = demo_sub.add_parser("pick", help="side-pose pick from layer")
    pick.add_argument("--src", type=positive_layer, required=True)
    place = demo_sub.add_parser("place", help="side-pose place to layer")
    place.add_argument("--dst", type=positive_layer, required=True)
    transfer = demo_sub.add_parser("transfer", help="side-pose transfer between layers")
    transfer.add_argument("--src", type=positive_layer, required=True)
    transfer.add_argument("--dst", type=positive_layer, required=True)
    transfer.add_argument("--variant", choices=DEMO_VARIANTS.keys(), default="auto")
    demo_sub.add_parser("key-turn", help="run key-turn demo macro")
    demo_sub.add_parser("cabinet-pull", help="run cabinet-pull demo macro")
    demo_sub.add_parser("home", help="run demo home chain")
    demo_sub.add_parser("stop", help="abort active demo")

    arm = sub.add_parser("arm", help="single arm commands")
    arm_sub = arm.add_subparsers(dest="arm_command", required=True)
    arm_sub.add_parser("home")
    arm_sub.add_parser("stop")
    xyz = arm_sub.add_parser("xyz")
    xyz.add_argument("x", type=float)
    xyz.add_argument("y", type=float)
    xyz.add_argument("z", type=float)
    pose = arm_sub.add_parser("pose")
    pose.add_argument("x", type=float)
    pose.add_argument("y", type=float)
    pose.add_argument("z", type=float)
    pose.add_argument("roll", type=float)
    pose.add_argument("pitch", type=float)

    grip = sub.add_parser("grip", help="set gripper angle in radians")
    grip.add_argument("rad", help="angle in radians, or preset: open/close")

    top = sub.add_parser("top", help="send named top-pose tuning point from profile")
    top.add_argument(
        "pose",
        choices=(
            "pick_approach",
            "pick_down",
            "pick_lift",
            "place_approach",
            "place_down",
            "place_lift",
        ),
    )

    side = sub.add_parser("side", help="send named side-pose tuning point from profile")
    side.add_argument(
        "pose",
        choices=(
            "safe",
            "pick_approach",
            "pick_clearance",
            "pick_pre_insert",
            "pick_insert",
            "carry_safe",
            "place_approach",
            "place_clearance",
            "place_pre_insert",
            "place_insert",
            "retreat_safe",
        ),
    )

    lift = sub.add_parser("lift", help="single lift commands")
    lift_sub = lift.add_subparsers(dest="lift_command", required=True)
    lift_sub.add_parser("home")
    lift_sub.add_parser("stop")
    lift_z = lift_sub.add_parser("z")
    lift_z.add_argument("z", type=float)
    lift_layer = lift_sub.add_parser("layer")
    lift_layer.add_argument("layer", type=positive_layer)

    sub.add_parser("stop", help="stop chassis, arm, lift, and demo")
    sub.add_parser("static-seq", help="run full static top-pose sequence from profile")
    sub.add_parser("cabinet-pull", help="top-pose cabinet clamp + chassis backward pull")
    sub.add_parser("key-turn", help="top-pose key grip + rotate 90deg + chassis backward pull")
    side_transfer = sub.add_parser("side-transfer", help="run side-pose transfer sequence from profile")
    side_transfer.add_argument("--src", type=positive_layer, required=True)
    side_transfer.add_argument("--dst", type=positive_layer, required=True)
    side_pick = sub.add_parser("side-pick", help="lift to a layer and test side-pose grasp")
    side_pick.add_argument("--layer", type=positive_layer, required=True)
    export = sub.add_parser("export-config", help="write tuned profile values into firmware config")
    export.add_argument("--config", type=Path, default=CONFIG_PATH)
    sub.add_parser("monitor", help="only read incoming telemetry")
    sub.add_parser("repl", help="interactive command loop")
    return parser


def run_args(console: DemoConsole, args: argparse.Namespace, profile: dict) -> None:
    if args.command == "demo":
        if args.demo_command == "static":
            console.demo_run(DEMO_ID_STATIC_PICK_PLACE)
        elif args.demo_command == "pick":
            console.demo_run(DEMO_ID_LAYER_PICK, src_layer=args.src)
        elif args.demo_command == "place":
            console.demo_run(DEMO_ID_LAYER_PLACE, dst_layer=args.dst)
        elif args.demo_command == "transfer":
            console.demo_run(
                DEMO_ID_LAYER_TRANSFER,
                src_layer=args.src,
                dst_layer=args.dst,
                variant=DEMO_VARIANTS[args.variant],
            )
        elif args.demo_command == "key-turn":
            console.demo_run(DEMO_ID_KEY_TURN)
        elif args.demo_command == "cabinet-pull":
            console.demo_run(DEMO_ID_CABINET_PULL)
        elif args.demo_command == "home":
            console.demo_home()
        elif args.demo_command == "stop":
            console.demo_stop()
    elif args.command == "arm":
        if args.arm_command == "home":
            console.arm_home(confirmed=args.confirmed)
        elif args.arm_command == "stop":
            console.arm_stop()
        elif args.arm_command == "xyz":
            console.arm_xyz(args.x, args.y, args.z, confirmed=args.confirmed)
        elif args.arm_command == "pose":
            console.arm_pose(args.x, args.y, args.z, args.roll, args.pitch, confirmed=args.confirmed)
    elif args.command == "grip":
        if args.rad in ("open", "close"):
            console.gripper(gripper_from_profile(profile, args.rad), confirmed=args.confirmed)
        else:
            console.gripper(float(args.rad), confirmed=args.confirmed)
    elif args.command == "top":
        console.arm_pose(*top_pose_from_profile(profile, args.pose), confirmed=args.confirmed)
    elif args.command == "side":
        console.arm_pose(*side_pose_from_profile(profile, args.pose), confirmed=args.confirmed)
    elif args.command == "lift":
        if args.lift_command == "home":
            console.lift_home(confirmed=args.confirmed)
        elif args.lift_command == "stop":
            console.lift_stop()
        elif args.lift_command == "z":
            console.lift_z(args.z, confirmed=args.confirmed)
        elif args.lift_command == "layer":
            console.lift_layer(profile, args.layer, confirmed=args.confirmed)
    elif args.command == "stop":
        console.demo_stop()
        console.chassis_stop()
        console.arm_stop()
        console.lift_stop()
    elif args.command == "static-seq":
        console.run_static_top_sequence(profile, args.step_delay, confirmed=args.confirmed or True)
    elif args.command == "side-transfer":
        console.run_side_transfer_sequence(profile, args.src, args.dst, args.step_delay, confirmed=args.confirmed)
    elif args.command == "side-pick":
        console.run_side_pick_sequence(profile, args.layer, args.step_delay, confirmed=args.confirmed)
    elif args.command == "cabinet-pull":
        console.run_cabinet_pull_sequence(profile, args.step_delay, confirmed=True)
    elif args.command == "key-turn":
        console.run_key_turn_sequence(profile, args.step_delay, confirmed=True)
    elif args.command == "export-config":
        write_profile_to_config(profile, args.config)
    elif args.command == "monitor":
        console.read_frames(args.monitor if args.monitor > 0 else 5.0)


def repl_help() -> None:
    print("Commands:")
    print("  demo static")
    print("  demo pick <src_layer>")
    print("  demo place <dst_layer>")
    print("  demo transfer <src_layer> <dst_layer> [auto|down|up|0|1|2]")
    print("  demo key-turn")
    print("  demo cabinet-pull")
    print("  demo home | demo stop")
    print("  arm home | arm stop")
    print("  pose <x> <y> <z> <roll> <pitch>")
    print("  top pick_approach|pick_down|pick_lift|place_approach|place_down|place_lift")
    print(
        "  side safe|pick_approach|pick_clearance|pick_pre_insert|pick_insert|"
        "carry_safe|place_approach|place_clearance|place_pre_insert|place_insert|retreat_safe"
    )
    print("  xyz <x> <y> <z>")
    print("  grip <rad>|open|close")
    print("  lift home | lift stop | lift z <z> | lift layer <1|2|3>")
    print("  stop")
    print("  static-seq")
    print("  cabinet-pull")
    print("  key-turn")
    print("  side-transfer <src_layer> <dst_layer>")
    print("  side-pick <layer>")
    print("  confirmed <command>")
    print("  set-layer <layer> <z_m>")
    print("  set-layer-dir <up|down> <layer> <z_m>")
    print("  set-gripper <open|close> <rad>")
    print("  set-pick-lift <z_m>")
    print("  set-cabinet-lift <z_m>")
    print("  set-key-lift <z_m>")
    print("  set-top-pose <name> <x> <y> <z> <roll> <pitch>")
    print("  reload")
    print("  export-config")
    print("  monitor <seconds>")
    print("  quit")


def parse_layer_token(token: str) -> int:
    layer = int(token)
    if layer not in (1, 2, 3):
        raise ValueError("layer must be 1, 2, or 3")
    return layer


def parse_demo_variant_token(token: str) -> int:
    if token in DEMO_VARIANTS:
        return DEMO_VARIANTS[token]
    variant = int(token, 0)
    if variant not in DEMO_VARIANTS.values():
        raise ValueError("variant must be auto/0, down/1, or up/2")
    return variant


def run_repl(console: DemoConsole, profile: dict, profile_path: Path) -> None:
    repl_help()
    while True:
        try:
            line = input("demo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        parts = line.split()
        confirmed = False
        if parts[0] == "confirmed":
            confirmed = True
            parts = parts[1:]
            if not parts:
                print("usage: confirmed <command>")
                continue
        try:
            if parts[0] in ("quit", "exit"):
                return
            if parts[0] == "help":
                repl_help()
            elif parts[0] == "demo" and len(parts) >= 2:
                if parts[1] == "static":
                    console.demo_run(DEMO_ID_STATIC_PICK_PLACE)
                elif parts[1] == "pick" and len(parts) == 3:
                    console.demo_run(DEMO_ID_LAYER_PICK, src_layer=parse_layer_token(parts[2]))
                elif parts[1] == "place" and len(parts) == 3:
                    console.demo_run(DEMO_ID_LAYER_PLACE, dst_layer=parse_layer_token(parts[2]))
                elif parts[1] == "key-turn":
                    console.demo_run(DEMO_ID_KEY_TURN)
                elif parts[1] == "cabinet-pull":
                    console.demo_run(DEMO_ID_CABINET_PULL)
                elif parts[1] == "transfer" and len(parts) in (4, 5):
                    variant = parse_demo_variant_token(parts[4]) if len(parts) == 5 else DEMO_VARIANTS["auto"]
                    console.demo_run(
                        DEMO_ID_LAYER_TRANSFER,
                        src_layer=parse_layer_token(parts[2]),
                        dst_layer=parse_layer_token(parts[3]),
                        variant=variant,
                    )
                elif parts[1] == "home":
                    console.demo_home()
                elif parts[1] == "stop":
                    console.demo_stop()
                else:
                    print("bad demo command")
            elif parts[0] == "arm" and len(parts) == 2:
                if parts[1] == "home":
                    console.arm_home(confirmed=confirmed)
                elif parts[1] == "stop":
                    console.arm_stop()
                else:
                    print("bad arm command")
            elif parts[0] == "pose" and len(parts) == 6:
                console.arm_pose(*(float(value) for value in parts[1:6]), confirmed=confirmed)
            elif parts[0] == "top" and len(parts) == 2:
                console.arm_pose(*top_pose_from_profile(profile, parts[1]), confirmed=confirmed)
            elif parts[0] == "side" and len(parts) == 2:
                console.arm_pose(*side_pose_from_profile(profile, parts[1]), confirmed=confirmed)
            elif parts[0] == "xyz" and len(parts) == 4:
                console.arm_xyz(*(float(value) for value in parts[1:4]), confirmed=confirmed)
            elif parts[0] == "grip" and len(parts) == 2:
                if parts[1] in ("open", "close"):
                    console.gripper(gripper_from_profile(profile, parts[1]), confirmed=confirmed)
                else:
                    console.gripper(float(parts[1]), confirmed=confirmed)
            elif parts[0] == "lift" and len(parts) >= 2:
                if parts[1] == "home":
                    console.lift_home(confirmed=confirmed)
                elif parts[1] == "stop":
                    console.lift_stop()
                elif parts[1] == "z" and len(parts) == 3:
                    console.lift_z(float(parts[2]), confirmed=confirmed)
                elif parts[1] == "layer" and len(parts) == 3:
                    console.lift_layer(profile, parse_layer_token(parts[2]), confirmed=confirmed)
                else:
                    print("bad lift command")
            elif parts[0] == "stop":
                console.demo_stop()
                console.chassis_stop()
                console.arm_stop()
                console.lift_stop()
            elif parts[0] == "static-seq":
                console.run_static_top_sequence(profile, 0.0, confirmed=True)
            elif parts[0] == "cabinet-pull":
                console.run_cabinet_pull_sequence(profile, 0.0, confirmed=True)
            elif parts[0] == "key-turn":
                console.run_key_turn_sequence(profile, 0.0, confirmed=True)
            elif parts[0] == "side-transfer" and len(parts) == 3:
                confirmed = True
                console.run_side_transfer_sequence(
                    profile,
                    parse_layer_token(parts[1]),
                    parse_layer_token(parts[2]),
                    0.0,
                    confirmed=confirmed,
                )
            elif parts[0] == "side-pick" and len(parts) == 2:
                confirmed = True
                console.run_side_pick_sequence(profile, parse_layer_token(parts[1]), 0.0, confirmed=confirmed)
            elif parts[0] == "set-layer" and len(parts) == 3:
                set_layer_height(profile, profile_path, parse_layer_token(parts[1]), float(parts[2]))
            elif parts[0] == "set-layer-dir" and len(parts) == 4:
                set_layer_direction_height(
                    profile,
                    profile_path,
                    parts[1],
                    parse_layer_token(parts[2]),
                    float(parts[3]),
                )
            elif parts[0] == "set-gripper" and len(parts) == 3:
                set_gripper_preset(profile, profile_path, parts[1], float(parts[2]))
            elif parts[0] == "set-pick-lift" and len(parts) == 2:
                set_side_pick_lift_z(profile, profile_path, float(parts[1]))
            elif parts[0] == "set-cabinet-lift" and len(parts) == 2:
                set_cabinet_lift_z(profile, profile_path, float(parts[1]))
            elif parts[0] == "set-key-lift" and len(parts) == 2:
                set_key_lift_z(profile, profile_path, float(parts[1]))
            elif parts[0] == "set-top-pose" and len(parts) == 7:
                set_top_pose(profile, profile_path, parts[1], *[float(v) for v in parts[2:7]])
            elif parts[0] == "reload":
                profile.clear()
                profile.update(load_profile(profile_path))
                print(f"reloaded {profile_path}")
            elif parts[0] == "export-config":
                write_profile_to_config(profile)
            elif parts[0] == "monitor" and len(parts) == 2:
                console.read_frames(float(parts[1]))
            else:
                print("unknown command")
        except Exception as exc:  # keep REPL alive during tuning
            if confirmed:
                console.demo_stop()
                console.chassis_stop()
                console.arm_stop()
                console.lift_stop()
            print(f"error: {exc}")


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    source = CTRL_SRC_BT if args.source == "bt" else CTRL_SRC_HOST
    profile = load_profile(args.profile)

    try:
        if args.command == "export-config":
            write_profile_to_config(profile, args.config)
            return 0
        with DemoConsole(args.port, args.baudrate, source, dry_run=args.dry_run) as console:
            try:
                if args.command == "repl":
                    run_repl(console, profile, args.profile)
                else:
                    run_args(console, args, profile)
                if args.monitor > 0 and args.command != "monitor":
                    console.read_frames(args.monitor)
            except Exception:
                if (args.confirmed or args.command in ("side-pick", "side-transfer", "cabinet-pull", "key-turn")) and not args.dry_run:
                    console.demo_stop()
                    console.chassis_stop()
                    console.arm_stop()
                    console.lift_stop()
                raise
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
