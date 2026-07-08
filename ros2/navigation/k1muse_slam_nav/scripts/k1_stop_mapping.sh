#!/usr/bin/env bash
set -uo pipefail

STATE_DIR="/home/bianbu/.ros/k1muse_slam_nav"
RUN_DIR="${STATE_DIR}/run"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
source /home/bianbu/k1muse_slam_ros/install/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-save}"
MCU_BRIDGE_PREFIX="$(ros2 pkg prefix k1muse_mcu_bridge)"
MCU_BRIDGE_SCRIPT="${MCU_BRIDGE_PREFIX}/lib/k1muse_mcu_bridge/k1_stop_mcu_bridge.sh"

"${SCRIPT_DIR}/k1_stop_map.sh" "${MODE}"
"${MCU_BRIDGE_SCRIPT}"

echo "Mapping stop done."
exit 0
