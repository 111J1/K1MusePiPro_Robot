#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
source /home/bianbu/k1muse_slam_ros/install/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MCU_BRIDGE_PREFIX="$(ros2 pkg prefix k1muse_mcu_bridge)"
MCU_BRIDGE_SCRIPT="${MCU_BRIDGE_PREFIX}/lib/k1muse_mcu_bridge/k1_start_mcu_bridge.sh"

"${MCU_BRIDGE_SCRIPT}"
K1_SKIP_ODOM_PREFLIGHT=1 "${SCRIPT_DIR}/k1_start_map.sh"

echo "Mapping launch started"
