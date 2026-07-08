#!/usr/bin/env bash
set -uo pipefail

STATE_DIR="/home/bianbu/.ros/k1muse_slam_nav"
RUN_DIR="${STATE_DIR}/run"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-save}"

REMOTE_COMPUTE="${K1_REMOTE_COMPUTE:-1}"
REMOTE_HOST="${K1_REMOTE_SLAM_HOST:-user@10.10.10.1}"
REMOTE_STATE_DIR="${K1_REMOTE_STATE_DIR:-/home/user/.ros/k1muse_slam_nav}"
REMOTE_MARKER="${RUN_DIR}/remote_slam_mapping.pid"

if [ "${MODE}" = "fast" ]; then
  echo "Fast map stop requested; skipping automatic map save."
else
  set +u
  source /opt/ros/humble/setup.bash
source /home/bianbu/k1muse_communicate_ros/install/setup.bash
source /home/bianbu/k1muse_slam_ros/install/setup.bash
set -u
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  if [ -z "${CYCLONEDDS_URI:-}" ] && [ -f "${HOME}/.ros/cyclonedds_end.xml" ]; then
    export CYCLONEDDS_URI="file://${HOME}/.ros/cyclonedds_end.xml"
  fi
  export ROS_DISABLE_DAEMON=true

  SAVE_OUTPUT="$("${SCRIPT_DIR}/k1_save_map.sh" stop 2>&1)"
  SAVE_RC=$?
  if [ "${SAVE_RC}" -eq 0 ]; then
    echo "${SAVE_OUTPUT}"
  else
    echo "MAP_SAVE_FAILED=${SAVE_RC}"
    echo "${SAVE_OUTPUT}"
  fi
fi

stop_remote_mapping() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_HOST}" \
    "REMOTE_STATE_DIR='${REMOTE_STATE_DIR}' bash -s" <<'REMOTE' || true
set -uo pipefail
RUN_DIR="${REMOTE_STATE_DIR}/run"
for pattern in \
  'ros2 launch k1muse_slam_nav slam\.launch\.py' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_node' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node'
do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done
sleep 2
for pattern in \
  'ros2 launch k1muse_slam_nav slam\.launch\.py' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_node' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node'
do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done
rm -f "${RUN_DIR}/slam_mapping.pid"
REMOTE
  rm -f "${REMOTE_MARKER}"
}

if [ "${REMOTE_COMPUTE}" = "1" ]; then
  stop_remote_mapping
fi

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

add_pidfile_tree "${RUN_DIR}/slam_mapping.pid" '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch k1muse_slam_nav slam\.launch\.py use_rviz:=false enable_imu:=(true|false)$'
add_pidfile_tree "${RUN_DIR}/sensors.pid" 'ros2 launch k1muse_slam_nav _sensors\.launch\.py'

add_matching_trees '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch k1muse_slam_nav slam\.launch\.py use_rviz:=false enable_imu:=(true|false)$'
add_matching_trees 'ros2 launch k1muse_slam_nav slam\.launch\.py'
add_matching_trees 'ros2 launch k1muse_slam_nav _sensors\.launch\.py'
add_matching_trees '^/home/bianbu/k1muse_slam_ros/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node($| )'
add_matching_trees '^/home/bianbu/k1muse_slam_ros/install/imu_ros2_device/lib/imu_ros2_device/ybimu_driver($| )'
add_matching_trees '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher($| )'
add_matching_trees '^/opt/ros/humble/lib/cartographer_ros/cartographer_node .*k1muse_slam_nav'
add_matching_trees '^/opt/ros/humble/lib/cartographer_ros/cartographer_node($| )'
add_matching_trees '^/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node($| )'

if [ -z "${PIDS// /}" ]; then
  echo "No map processes matched."
else
  echo "Stopping map PIDs:${PIDS}"
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

rm -f "${RUN_DIR}/slam_mapping.pid" "${RUN_DIR}/sensors.pid" "${REMOTE_MARKER}"
echo "Map stop done."
exit 0
