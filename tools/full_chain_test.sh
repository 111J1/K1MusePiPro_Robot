#!/bin/bash
# Full chain: start ALL nodes → verify ALL topics/frames/data
# Requires MCU simulator running on PC:  python tools/mcu_sim.py COM6 --json
set -e

export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/bros/humble/setup.bash
source /home/bianbu/k1muse_slam_ros/install/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash

P=0; F=0
ok()   { echo "  ✅ $1"; P=$((P+1)); }
fail() { echo "  ❌ $1"; F=$((F+1)); }

trap 'kill $BRIDGE_PID $ADAPTER_PID $LIDAR_PID $IMU_PID $RSP_PID 2>/dev/null' EXIT

echo "══════════════════════════════════════════"
echo " Full Chain Integration Test"
echo " ENV: DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
echo "══════════════════════════════════════════"

# ── 1. Start mcu_bridge ──
echo ""
echo "── 1. mcu_bridge ──"
ros2 run k1muse_mcu_bridge mcu_bridge_node > /tmp/mcu_bridge_chain.log 2>&1 &
BRIDGE_PID=$!
sleep 4
grep -q "Serial port" /tmp/mcu_bridge_chain.log && ok "port opened" || fail "port not opened"
kill -0 $BRIDGE_PID 2>/dev/null && ok "process alive" || fail "process dead"

# ── 2. Start chassis_adapter ──
echo ""
echo "── 2. chassis_adapter ──"
ros2 run k1muse_chassis_adapter adapter_node \
  --ros-args --params-file $(ros2 pkg prefix k1muse_chassis_adapter)/share/k1muse_chassis_adapter/config/adapter.yaml \
  > /tmp/adapter_chain.log 2>&1 &
ADAPTER_PID=$!
sleep 5
grep -q "WCS odometry reset" /tmp/adapter_chain.log && ok "WCS reset done" || fail "WCS reset missing"
grep -q "OdomPublisher ready" /tmp/adapter_chain.log && ok "OdomPublisher ready" || fail "OdomPublisher"
grep -q "CmdVelRouter ready" /tmp/adapter_chain.log && ok "CmdVelRouter ready" || fail "CmdVelRouter"

# ── 3. Start lidar ──
echo ""
echo "── 3. Lidar ──"
ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -p product_name:=LDLiDAR_LD06 \
  -p topic_name:=scan -p frame_id:=base_laser \
  -p port_name:=/dev/mylidar -p port_baudrate:=230400 \
  -p laser_scan_dir:=true \
  > /tmp/lidar_chain.log 2>&1 &
LIDAR_PID=$!
sleep 4
kill -0 $LIDAR_PID 2>/dev/null && ok "lidar alive" || fail "lidar dead"

# ── 4. Start IMU ──
echo ""
echo "── 4. IMU ──"
ros2 run imu_ros2_device ybimu_driver --ros-args \
  -p port:=/dev/myimu -p frame_id:=imu_link \
  > /tmp/imu_chain.log 2>&1 &
IMU_PID=$!
sleep 4
kill -0 $IMU_PID 2>/dev/null && ok "IMU alive" || fail "IMU dead"

# ── 5. Start RSP ──
echo ""
echo "── 5. RSP ──"
URDF=$(xacro $(ros2 pkg prefix k1muse_description)/share/k1muse_description/urdf/musepi_robot.urdf.xacro 2>/dev/null)
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$URDF" \
  > /tmp/rsp_chain.log 2>&1 &
RSP_PID=$!
sleep 2
grep -q "got segment base_link" /tmp/rsp_chain.log && ok "RSP segments" || fail "RSP segments"

# ── 6. Topic inventory ──
echo ""
echo "── 6. Topic inventory ──"
TOPICS=$(ros2 topic list 2>&1)
echo "$TOPICS" | sort

for t in /mcu/chassis/status /mcu/chassis/mov /odom /scan /imu/data /cmd_vel /tf; do
  echo "$TOPICS" | grep -qx "$t" && ok "topic $t" || fail "topic $t missing"
done

# ── 7. Data payload checks ──
echo ""
echo "── 7. Data payloads ──"

# /mcu/chassis/status — from real MCU via CH340 simulator
echo "7a. /mcu/chassis/status (MCU→ROS):"
S=$(timeout 5 ros2 topic echo /mcu/chassis/status --once 2>&1)
if echo "$S" | grep -q "tick_ms"; then
  echo "$S" | head -14
  ok "/mcu/chassis/status has data"
else
  echo "NO DATA (mcu_sim running on PC?)"
  fail "/mcu/chassis/status no data"
fi

# /odom
echo ""
echo "7b. /odom:"
O=$(timeout 5 ros2 topic echo /odom --once 2>&1)
if echo "$O" | grep -q "twist"; then
  echo "$O" | head -16
  ok "/odom has data"
else
  fail "/odom no data"
fi

# /scan
echo ""
echo "7c. /scan:"
L=$(timeout 5 ros2 topic echo /scan --once 2>&1 | head -10)
if echo "$L" | grep -q "ranges"; then
  echo "$L"
  ok "/scan has data"
else
  fail "/scan no data"
fi

# /imu/data
echo ""
echo "7d. /imu/data:"
I=$(timeout 5 ros2 topic echo /imu/data --once 2>&1 | head -10)
if echo "$I" | grep -q "orientation"; then
  echo "$I"
  ok "/imu/data has data"
else
  fail "/imu/data no data"
fi

# ── 8. TF tree ──
echo ""
echo "── 8. TF tree ──"
for pair in "odom:base_footprint" "base_footprint:base_link" "base_link:base_laser" "base_link:imu_link"; do
  parent="${pair%%:*}"
  child="${pair##*:}"
  V=$(timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | head -4)
  if echo "$V" | grep -q "Translation"; then
    echo "$V" | head -4
    ok "  TF $parent→$child OK"
  else
    echo "$V" | head -2
    fail "  TF $parent→$child FAIL"
  fi
done

# ── 9. cmd_vel → MOV roundtrip ──
echo ""
echo "── 9. cmd_vel → MOV ──"
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.05}}" \
  > /dev/null 2>&1 &
sleep 4
M=$(timeout 5 ros2 topic echo /mcu/chassis/mov --once 2>&1)
if echo "$M" | grep -q "direction"; then
  echo "$M" | head -6
  DIR=$(echo "$M" | grep -oP 'direction: \K[0-9.]+')
  V=$(echo "$M" | grep -oP 'v: \K[0-9.]+')
  OM=$(echo "$M" | grep -oP 'omega: \K[0-9.]+')
  ok "MOV received: dir=$DIR v=$V omega=$OM"
else
  fail "MOV not received"
fi
kill %1 2>/dev/null || true

# ── 10. Frequencies ──
echo ""
echo "── 10. Frequencies ──"
for T in /odom /scan /imu/data; do
  HZ=$(timeout 8 ros2 topic hz "$T" 2>&1 | grep -oP 'average rate: \K[0-9.]+' | tail -1)
  [ -n "$HZ" ] && ok "  $T ≈ ${HZ}Hz" || fail "  $T rate unknown"
done

# ── Summary ──
echo ""
echo "══════════════════════════════════════════"
echo " Results: $P passed, $F failed  (total $((P+F)))"
echo "══════════════════════════════════════════"

