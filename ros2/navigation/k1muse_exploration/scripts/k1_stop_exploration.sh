#!/usr/bin/env bash
set -uo pipefail

MODE="${1:-fast}"

STATE_DIR="/home/bianbu/.ros/k1muse_exploration"
RUN_DIR="${STATE_DIR}/run"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

SLAM_STATE_DIR="/home/bianbu/.ros/k1muse_slam_nav"
SLAM_RUN_DIR="${SLAM_STATE_DIR}/run"

REMOTE_COMPUTE="${K1_REMOTE_COMPUTE:-1}"
REMOTE_HOST="${K1_REMOTE_SLAM_HOST:-user@10.10.10.1}"
REMOTE_STATE_DIR="${K1_REMOTE_EXPLORATION_STATE_DIR:-/home/user/.ros/k1muse_exploration}"
REMOTE_MARKER="${RUN_DIR}/remote_exploration.pid"

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

if [ "${MODE}" = "save" ]; then
  SLAM_NAV_PREFIX="$(ros2 pkg prefix k1muse_slam_nav)"
  "${SLAM_NAV_PREFIX}/lib/k1muse_slam_nav/k1_save_map.sh" manual || true
fi

timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true

stop_remote_exploration() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_HOST}" \
    "REMOTE_STATE_DIR='${REMOTE_STATE_DIR}' bash -s" <<'REMOTE' || true
set -uo pipefail
RUN_DIR="${REMOTE_STATE_DIR}/run"
for pattern in \
  'ros2 launch k1muse_exploration explore\.launch\.py' \
  'rrt_frontier_explorer' \
  'nav2_' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_node' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node'
do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done
sleep 2
for pattern in \
  'ros2 launch k1muse_exploration explore\.launch\.py' \
  'rrt_frontier_explorer' \
  'nav2_' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_node' \
  '/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node'
do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done
rm -f "${RUN_DIR}/exploration.pid"
REMOTE
  rm -f "${REMOTE_MARKER}"
}

if [ "${REMOTE_COMPUTE}" = "1" ]; then
  stop_remote_exploration
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

add_pidfile_tree "${RUN_DIR}/exploration.pid" 'ros2 launch k1muse_exploration explore\.launch\.py'
add_pidfile_tree "${SLAM_RUN_DIR}/sensors.pid" 'ros2 launch k1muse_slam_nav _sensors\.launch\.py'
add_matching_trees 'ros2 launch k1muse_exploration explore\.launch\.py'
add_matching_trees 'ros2 launch k1muse_slam_nav _sensors\.launch\.py'
add_matching_trees '^/home/bianbu/k1muse_slam_ros/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node($| )'
add_matching_trees '^/home/bianbu/k1muse_slam_ros/install/imu_ros2_device/lib/imu_ros2_device/ybimu_driver($| )'
add_matching_trees '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher($| )'

if [ -z "${PIDS// /}" ]; then
  echo "No K1 exploration processes matched."
else
  echo "Stopping K1 exploration PIDs:${PIDS}"
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

rm -f "${RUN_DIR}/exploration.pid" "${SLAM_RUN_DIR}/sensors.pid" "${REMOTE_MARKER}"

MCU_BRIDGE_PREFIX="$(ros2 pkg prefix k1muse_mcu_bridge 2>/dev/null || true)"
if [ -n "${MCU_BRIDGE_PREFIX}" ] && [ -x "${MCU_BRIDGE_PREFIX}/lib/k1muse_mcu_bridge/k1_stop_mcu_bridge.sh" ]; then
  "${MCU_BRIDGE_PREFIX}/lib/k1muse_mcu_bridge/k1_stop_mcu_bridge.sh" || true
fi

echo "K1 exploration stop done."
exit 0
