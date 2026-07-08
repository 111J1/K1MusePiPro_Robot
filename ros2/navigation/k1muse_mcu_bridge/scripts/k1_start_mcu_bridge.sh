#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="/home/bianbu/.ros/k1muse_mcu_bridge"
RUN_DIR="${STATE_DIR}/run"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
if [ -z "${CYCLONEDDS_URI:-}" ] && [ -f "${HOME}/.ros/cyclonedds_end.xml" ]; then
  export CYCLONEDDS_URI="file://${HOME}/.ros/cyclonedds_end.xml"
fi

process_exists() {
  pgrep -f "$1" >/dev/null 2>&1
}

write_pid() {
  local name="$1"
  local pid="$2"
  echo "${pid}" >"${RUN_DIR}/${name}.pid"
}

process_ready() {
  process_exists '^/usr/bin/python3 /opt/ros/humble/bin/ros2 run k1muse_mcu_bridge mcu_bridge_node' ||
    process_exists '/home/bianbu/k1muse_communicate_ros/install/k1muse_mcu_bridge/lib/k1muse_mcu_bridge/mcu_bridge_node'
}

log_ready() {
  grep -q "Serial port /dev/mymcu opened" "${LOG_DIR}/mcu_bridge.log" 2>/dev/null
}

serial_has_data() {
  if [ ! -e /dev/mymcu ]; then
    echo "/dev/mymcu does not exist" >&2
    return 1
  fi
  stty -F /dev/mymcu 115200 raw -echo >/dev/null 2>&1 || return 1
  timeout 3 dd if=/dev/mymcu of=/dev/null bs=1 count=1 status=none
}

if process_ready; then
  echo "mcu_bridge_node already running"
else
  echo "checking /dev/mymcu data before starting mcu_bridge_node"
  if ! serial_has_data; then
    echo "No data observed on /dev/mymcu; not starting mcu_bridge_node" >&2
    exit 1
  fi

  echo "starting mcu_bridge_node"
  : >"${LOG_DIR}/mcu_bridge.log"
  nohup ros2 run k1muse_mcu_bridge mcu_bridge_node --ros-args -p enable_cmd_vel_output:=true >"${LOG_DIR}/mcu_bridge.log" 2>&1 &
  write_pid "mcu_bridge" "$!"
fi

deadline=$((SECONDS + 20))
while [ "${SECONDS}" -lt "${deadline}" ]; do
  if process_ready && log_ready; then
    sleep 1
    echo "MCU bridge started"
    exit 0
  fi
  sleep 0.25
done

echo "MCU bridge start timed out; see ${LOG_DIR}/mcu_bridge.log" >&2
"${SCRIPT_DIR}/k1_stop_mcu_bridge.sh" >/dev/null 2>&1 || true
exit 1
