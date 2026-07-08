#!/usr/bin/env bash
set -uo pipefail

STATE_DIR="/home/bianbu/.ros/k1muse_mcu_bridge"
RUN_DIR="${STATE_DIR}/run"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

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

timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true

PIDS=""

add_pid() {
  local pid="${1:-}"
  [ -n "${pid}" ] || return 0
  case " ${PIDS} " in
    *" ${pid} "*) ;;
    *) PIDS="${PIDS} ${pid}" ;;
  esac
}

cmdline() {
  ps -p "$1" -o args= 2>/dev/null || true
}

collect_tree() {
  local pid="$1"
  kill -0 "${pid}" 2>/dev/null || return 0
  local child
  for child in $(ps -eo pid=,ppid= | awk -v p="${pid}" '$2 == p {print $1}'); do
    collect_tree "${child}"
  done
  add_pid "${pid}"
}

add_pidfile_tree() {
  local file="$1"
  local expected_regex="$2"
  [ -f "${file}" ] || return 0
  local pid
  pid="$(tr -cd '0-9' <"${file}" | head -c 16)"
  [ -n "${pid}" ] || return 0
  if kill -0 "${pid}" 2>/dev/null && cmdline "${pid}" | grep -Eq "${expected_regex}"; then
    collect_tree "${pid}"
  fi
}

add_matching_trees() {
  local expected_regex="$1"
  local pid
  for pid in $(pgrep -f "${expected_regex}" 2>/dev/null || true); do
    [ "${pid}" != "$$" ] || continue
    cmdline "${pid}" | grep -Eq "${expected_regex}" && collect_tree "${pid}"
  done
}

add_pidfile_tree "${RUN_DIR}/mcu_bridge.pid" '^/usr/bin/python3 /opt/ros/humble/bin/ros2 run k1muse_mcu_bridge mcu_bridge_node --ros-args -p enable_cmd_vel_output:=true$'
add_matching_trees '^/usr/bin/python3 /opt/ros/humble/bin/ros2 run k1muse_mcu_bridge mcu_bridge_node( --ros-args -p enable_cmd_vel_output:=true)?$'
add_matching_trees '^/home/bianbu/k1muse_communicate_ros/install/k1muse_mcu_bridge/lib/k1muse_mcu_bridge/mcu_bridge_node($| )'

if [ -z "${PIDS// /}" ]; then
  echo "No MCU bridge processes matched."
else
  echo "Stopping MCU bridge PIDs:${PIDS}"
  for pid in ${PIDS}; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 2
  for pid in ${PIDS}; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
fi

rm -f "${RUN_DIR}/mcu_bridge.pid"
echo "MCU bridge stop done."
exit 0
