#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 4 ]; then
  echo "usage: k1_start_exploration.sh MIN_X MAX_X MIN_Y MAX_Y" >&2
  exit 2
fi

MIN_X="$1"
MAX_X="$2"
MIN_Y="$3"
MAX_Y="$4"

STATE_DIR="/home/bianbu/.ros/k1muse_exploration"
RUN_DIR="${STATE_DIR}/run"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

process_exists() {
  pgrep -f "$1" >/dev/null 2>&1
}

write_pid() {
  local name="$1"
  local pid="$2"
  echo "${pid}" >"${RUN_DIR}/${name}.pid"
}

REMOTE_COMPUTE="${K1_REMOTE_COMPUTE:-1}"
REMOTE_HOST="${K1_REMOTE_SLAM_HOST:-user@10.10.10.1}"
REMOTE_COMMS_WS="${K1_REMOTE_COMMS_WS:-/home/user/k1muse_communicate_ros}"
REMOTE_WS="${K1_REMOTE_SLAM_WS:-/home/user/k1muse_slam_ros}"
REMOTE_STATE_DIR="${K1_REMOTE_EXPLORATION_STATE_DIR:-/home/user/.ros/k1muse_exploration}"
REMOTE_MARKER="${RUN_DIR}/remote_exploration.pid"

if process_exists 'k1muse_exploration explore\.launch\.py'; then
  echo "K1 exploration already running"
  exit 0
fi

MCU_BRIDGE_PREFIX="$(ros2 pkg prefix k1muse_mcu_bridge)"
MCU_BRIDGE_SCRIPT="${MCU_BRIDGE_PREFIX}/lib/k1muse_mcu_bridge/k1_start_mcu_bridge.sh"
"${MCU_BRIDGE_SCRIPT}"

USE_RVIZ="${K1_EXPLORE_USE_RVIZ:-false}"
ENABLE_IMU="${K1_EXPLORE_ENABLE_IMU:-true}"
REQUIRE_ODOM_PREFLIGHT="${K1_EXPLORE_REQUIRE_ODOM_PREFLIGHT:-false}"

if [ "${REMOTE_COMPUTE}" = "1" ]; then
  SLAM_NAV_PREFIX="$(ros2 pkg prefix k1muse_slam_nav)"
  "${SLAM_NAV_PREFIX}/lib/k1muse_slam_nav/k1_start_sensors.sh"

  ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_HOST}" \
    "REMOTE_COMMS_WS='${REMOTE_COMMS_WS}' REMOTE_WS='${REMOTE_WS}' REMOTE_STATE_DIR='${REMOTE_STATE_DIR}' MIN_X='${MIN_X}' MAX_X='${MAX_X}' MIN_Y='${MIN_Y}' MAX_Y='${MAX_Y}' USE_RVIZ='${USE_RVIZ}' ENABLE_IMU='${ENABLE_IMU}' REQUIRE_ODOM_PREFLIGHT='${REQUIRE_ODOM_PREFLIGHT}' bash -s" <<'REMOTE'
set -euo pipefail
RUN_DIR="${REMOTE_STATE_DIR}/run"
LOG_DIR="${REMOTE_STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${REMOTE_COMMS_WS}/install/setup.bash"
source "${REMOTE_WS}/install/setup.bash"
	source /home/user/mppi_overlay/setup.bash 2>/dev/null || true
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
if [ -z "${CYCLONEDDS_URI:-}" ] && [ -f "${HOME}/.ros/cyclonedds_eth.xml" ]; then
  export CYCLONEDDS_URI="file://${HOME}/.ros/cyclonedds_eth.xml"
fi

if pgrep -f 'k1muse_exploration explore\.launch\.py' >/dev/null 2>&1; then
  echo "Remote K1 exploration already running"
  exit 0
fi

: >"${LOG_DIR}/exploration.log"
nohup ros2 launch k1muse_exploration explore.launch.py \
  use_rviz:="${USE_RVIZ}" \
  enable_imu:="${ENABLE_IMU}" \
  require_odom_precheck:="${REQUIRE_ODOM_PREFLIGHT}" \
  start_sensors:=false \
  send_goals:=true \
  min_x:="${MIN_X}" max_x:="${MAX_X}" min_y:="${MIN_Y}" max_y:="${MAX_Y}" \
  >"${LOG_DIR}/exploration.log" 2>&1 &
echo "$!" >"${RUN_DIR}/exploration.pid"

deadline=$((SECONDS + 45))
while [ "${SECONDS}" -lt "${deadline}" ]; do
  nodes="$(ros2 node list 2>/dev/null || true)"
  if printf '%s\n' "${nodes}" | grep -Fxq "/cartographer_node" &&
    printf '%s\n' "${nodes}" | grep -Fxq "/cartographer_occupancy_grid_node" &&
    printf '%s\n' "${nodes}" | grep -Fxq "/controller_server" &&
    printf '%s\n' "${nodes}" | grep -Fxq "/planner_server" &&
    printf '%s\n' "${nodes}" | grep -Fxq "/bt_navigator" &&
    printf '%s\n' "${nodes}" | grep -Fxq "/rrt_frontier_explorer"
  then
    echo "Remote K1 exploration started"
    exit 0
  fi
  sleep 0.5
done

echo "Remote K1 exploration start timed out; see ${LOG_DIR}/exploration.log" >&2
tail -n 120 "${LOG_DIR}/exploration.log" >&2 || true
exit 1
REMOTE
  echo "${REMOTE_HOST}" >"${REMOTE_MARKER}"
  exit 0
fi

echo "starting K1 exploration within x=[${MIN_X}, ${MAX_X}] y=[${MIN_Y}, ${MAX_Y}]"
: >"${LOG_DIR}/exploration.log"
nohup ros2 launch k1muse_exploration explore.launch.py \
  use_rviz:="${USE_RVIZ}" \
  enable_imu:="${ENABLE_IMU}" \
  require_odom_precheck:="${REQUIRE_ODOM_PREFLIGHT}" \
  send_goals:=true \
  min_x:="${MIN_X}" max_x:="${MAX_X}" min_y:="${MIN_Y}" max_y:="${MAX_Y}" \
  >"${LOG_DIR}/exploration.log" 2>&1 &
write_pid "exploration" "$!"

deadline=$((SECONDS + 45))
while [ "${SECONDS}" -lt "${deadline}" ]; do
  if ros2 node list 2>/dev/null | grep -Fxq "/rrt_frontier_explorer"; then
    echo "K1 exploration started"
    exit 0
  fi
  sleep 0.5
done

echo "K1 exploration start timed out; see ${LOG_DIR}/exploration.log" >&2
"${SCRIPT_DIR}/k1_stop_exploration.sh" fast >/dev/null 2>&1 || true
exit 1
