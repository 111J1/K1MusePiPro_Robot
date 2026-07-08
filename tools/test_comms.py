#!/usr/bin/env python3
"""
Full communication test: all Chassis commands + edge cases.
Starts mcu_bridge_node on Pi and mcu_sim on Windows, then runs test cases.

Usage:
  python tools/test_comms.py COM6
"""

import subprocess
import json
import time
import sys
import os
import tempfile

SSH = "ssh musepi"
ROS_SRC = "source /opt/ros/humble/setup.bash && source /home/bianbu/k1muse_communicate_ros/install/setup.bash"
SIM_OUT = os.path.join(tempfile.gettempdir(), "mcu_sim_test.jsonl")
SIM_LOG = "/tmp/mcu_bridge_test.log"
PROJECT = "E:/Embed_Game/MusePiPro_prj"
PYTHON = "C:/Users/YJY/AppData/Local/Programs/Python/Python313/python"

pass_cnt = 0
fail_cnt = 0
sim_proc = None


def run(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True,
                           text=True, timeout=timeout, cwd=PROJECT)
        return r.returncode, r.stdout.strip()
    except subprocess.TimeoutExpired:
        return -1, "TIMEOUT"


def ssh(cmd, timeout=15):
    return run(f'{SSH} "{cmd}"', timeout)


def pi_ros(cmd, timeout=15):
    return ssh(f"{ROS_SRC} && {cmd}", timeout)


def start_node():
    ssh("kill $(ps aux | grep 'mcu_bridge_node' | grep -v grep | awk '{print $2}') 2>/dev/null; sleep 0.3")
    ssh(f"setsid /home/bianbu/k1muse_communicate_ros/start_bridge.sh > {SIM_LOG} 2>&1 & disown")
    time.sleep(4)
    _, log = ssh(f"cat {SIM_LOG}")
    return "Serial port" in log


def start_simulator(port):
    global sim_proc
    if os.path.exists(SIM_OUT):
        os.remove(SIM_OUT)
    sim_proc = subprocess.Popen(
        [PYTHON, f"{PROJECT}/tools/mcu_sim.py", port, "--json"],
        stdout=open(SIM_OUT, "w"), stderr=subprocess.STDOUT)
    time.sleep(3)
    try:
        with open(SIM_OUT) as f:
            first = f.readline()
        return "MCU Simulator" in first
    except Exception:
        return False


def stop_simulator():
    global sim_proc
    if sim_proc:
        sim_proc.terminate()
        try:
            sim_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            sim_proc.kill()
        sim_proc = None
    time.sleep(1)


def read_sim_rx():
    frames = []
    try:
        with open(SIM_OUT) as f:
            for line in f:
                try:
                    evt = json.loads(line)
                    if evt.get("type") == "rx":
                        frames.append(evt["frame"])
                except json.JSONDecodeError:
                    pass
    except FileNotFoundError:
        pass
    return frames


def read_sim_tx_count():
    n = 0
    try:
        with open(SIM_OUT) as f:
            for line in f:
                if '"type": "tx"' in line:
                    n += 1
    except FileNotFoundError:
        pass
    return n


def check(condition, msg):
    global pass_cnt, fail_cnt
    if condition:
        pass_cnt += 1
        print(f"  PASS  {msg}")
    else:
        fail_cnt += 1
        print(f"  FAIL  {msg}")


def test_mov(desc, move_cs, direction, v, omega):
    pi_ros(
        f"ros2 topic pub --once /mcu/chassis/mov "
        f"k1muse_mcu_bridge/msg/ChassisMov "
        f"'{{move_cs: {move_cs}, direction: {direction}, v: {v}, omega: {omega}}}'")
    time.sleep(1.0)

    frames = read_sim_rx()
    mov_frames = [f for f in frames
                  if f.get("target") == "CHASSIS" and f.get("cmd") == "MOV"]
    last = mov_frames[-1] if mov_frames else None

    ok = (last is not None
          and last["payload"].get("move_cs") == move_cs
          and abs(last["payload"].get("direction", 999) - direction) < 0.001
          and abs(last["payload"].get("v", 999) - v) < 0.001
          and abs(last["payload"].get("omega", 999) - omega) < 0.001)
    check(ok, f"MOV {desc}: cs={move_cs} dir={direction} v={v} ω={omega}")
    return ok


def test_stop(desc):
    before = len([f for f in read_sim_rx()
                  if f.get("cmd") == "STOP"])
    pi_ros("ros2 service call /mcu/chassis/stop k1muse_mcu_bridge/srv/ChassisStop '{}'")
    time.sleep(1.0)
    after = len([f for f in read_sim_rx()
                 if f.get("cmd") == "STOP"])
    check(after > before, f"STOP {desc}: #{after} STOP frames")


def test_odom(desc, direction, x, y):
    pi_ros(
        f"ros2 service call /mcu/chassis/odom "
        f"k1muse_mcu_bridge/srv/ChassisOdom "
        f"'{{direction: {direction}, x: {x}, y: {y}}}'")
    time.sleep(1.0)

    frames = read_sim_rx()
    odom_frames = [f for f in frames
                   if f.get("target") == "CHASSIS" and f.get("cmd") == "ODOM"]
    last = odom_frames[-1] if odom_frames else None

    ok = (last is not None
          and abs(last["payload"].get("direction", 999) - direction) < 0.001
          and abs(last["payload"].get("x", 999) - x) < 0.001
          and abs(last["payload"].get("y", 999) - y) < 0.001)
    check(ok, f"ODOM {desc}: dir={direction} x={x} y={y}")
    return ok


# ═══════════════════════════════════════════════════════════════════════════════
def main():
    if len(sys.argv) < 2:
        print("Usage: python tools/test_comms.py COM6")
        sys.exit(1)
    port = sys.argv[1]

    print("=" * 56)
    print("Full Chassis Communication Test")
    print("=" * 56)

    # ── Setup ────────────────────────────────────────────────────────────────
    print("\n── Setup ──")

    ok = start_node()
    check(ok, "mcu_bridge_node started")
    if not ok:
        print("FATAL: node start failed")
        sys.exit(1)

    ok = start_simulator(port)
    check(ok, "MCU simulator started")
    if not ok:
        print("FATAL: simulator start failed")
        stop_simulator(); sys.exit(1)

    time.sleep(1)
    tx_n = read_sim_tx_count()
    check(tx_n > 5, f"Simulator TX status frames ({tx_n})")

    _, log = ssh(f"cat {SIM_LOG}")
    check("RX frame" in log, "Node RX status frames (MCU→ROS)")

    # ── ChassisMov ───────────────────────────────────────────────────────────
    print("\n── ChassisMov ──")
    test_mov("normal",           1,  0.5,  0.3,  0.0)
    test_mov("zero velocity",    1,  0.0,  0.0,  0.0)
    test_mov("LCS mode",         0,  0.0,  0.8,  0.0)
    test_mov("reverse",          1,  3.14, -0.2, -0.5)
    test_mov("max linear",       1,  0.0,  1.4,  0.0)
    test_mov("max angular",      1,  0.0,  0.0,  3.7)
    test_mov("negative dir",     1, -1.5,  0.3,  0.0)

    # ── ChassisStop ──────────────────────────────────────────────────────────
    print("\n── ChassisStop ──")
    test_stop("basic")
    test_stop("consecutive #1")
    test_stop("consecutive #2")
    test_stop("after MOV")
    pi_ros("ros2 topic pub --once /mcu/chassis/mov k1muse_mcu_bridge/msg/ChassisMov "
           "'{move_cs: 1, direction: 0.0, v: 0.5, omega: 0.0}'")
    time.sleep(0.3)
    test_stop("MOV→STOP")

    # ── ChassisOdom ──────────────────────────────────────────────────────────
    print("\n── ChassisOdom ──")
    test_odom("normal",        1.57,  0.5,  0.3)
    test_odom("zero",          0.0,   0.0,  0.0)
    test_odom("negative",     -3.14, -1.0, -0.5)
    test_odom("large values",  6.28,  100.0, 100.0)

    # ── Rapid Sequence ───────────────────────────────────────────────────────
    print("\n── Rapid Sequence ──")
    seq_before = len(read_sim_rx())
    test_mov("seq-1", 1, 0.1, 0.5, 0.0)
    test_mov("seq-2", 1, 0.2, 0.6, 0.0)
    pi_ros("ros2 service call /mcu/chassis/stop k1muse_mcu_bridge/srv/ChassisStop '{}'")
    time.sleep(0.3)
    pi_ros("ros2 service call /mcu/chassis/odom k1muse_mcu_bridge/srv/ChassisOdom "
           "'{direction: 1.0, x: 1.0, y: 1.0}'")
    time.sleep(1.0)
    seq_after = len(read_sim_rx())
    check(seq_after - seq_before >= 4, f"Rapid: +{seq_after-seq_before} frames in sequence")

    # ── SEQ Overflow ─────────────────────────────────────────────────────────
    print("\n── SEQ Counter ──")
    initial = len(read_sim_rx())
    for i in range(10):
        pi_ros("ros2 service call /mcu/chassis/stop k1muse_mcu_bridge/srv/ChassisStop '{}'")
        time.sleep(0.1)
    time.sleep(1.0)
    final = len(read_sim_rx())
    check(final - initial == 10, f"SEQ: {initial}→{final} (+{final-initial}, expected +10)")

    # ── Cleanup ──────────────────────────────────────────────────────────────
    print("\n── Cleanup ──")
    stop_simulator()
    ssh("kill $(ps aux | grep 'mcu_bridge_node' | grep -v grep | awk '{print $2}') 2>/dev/null")
    check(True, "done")

    # ── Summary ──────────────────────────────────────────────────────────────
    total = pass_cnt + fail_cnt
    print("\n" + "=" * 56)
    print(f"Results: {pass_cnt}/{total} passed  [{fail_cnt} failed]")
    print("=" * 56)
    return 0 if fail_cnt == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

