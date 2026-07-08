#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="/home/bianbu/.ros/k1muse_slam_nav"
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

REMOTE_COMPUTE="${K1_REMOTE_COMPUTE:-1}"
REMOTE_HOST="${K1_REMOTE_SLAM_HOST:-user@10.10.10.1}"
REMOTE_COMMS_WS="${K1_REMOTE_COMMS_WS:-/home/user/k1muse_communicate_ros}"
REMOTE_WS="${K1_REMOTE_SLAM_WS:-/home/user/k1muse_slam_ros}"
REMOTE_STATE_DIR="${K1_REMOTE_STATE_DIR:-/home/user/.ros/k1muse_slam_nav}"
REMOTE_MARKER="${RUN_DIR}/remote_slam_mapping.pid"

ros_node_exists() {
  ros2 node list 2>/dev/null | grep -Fxq "$1"
}

process_exists() {
  pgrep -f "$1" >/dev/null 2>&1
}

cartographer_ready() {
  ros_node_exists "/cartographer_node" ||
    process_exists '/opt/ros/humble/lib/cartographer_ros/cartographer_node'
}

topic_visible() {
  local topic="$1"
  timeout 1 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"
}

wait_topic_visible() {
  local topic="$1"
  local deadline=$((SECONDS + 15))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if topic_visible "${topic}"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

start_remote_mapping() {
  "${SCRIPT_DIR}/k1_start_sensors.sh"

  ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_HOST}" \
    "REMOTE_COMMS_WS='${REMOTE_COMMS_WS}' REMOTE_WS='${REMOTE_WS}' REMOTE_STATE_DIR='${REMOTE_STATE_DIR}' bash -s" <<'REMOTE'
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

if pgrep -f 'k1muse_slam_nav slam\.launch\.py' >/dev/null 2>&1; then
  echo "Remote Cartographer mapping already running"
  exit 0
fi

: >"${LOG_DIR}/slam_mapping.log"
nohup ros2 launch k1muse_slam_nav slam.launch.py \
  use_rviz:=false \
  enable_imu:=true \
  require_odom_precheck:=false \
  start_sensors:=false \
  >"${LOG_DIR}/slam_mapping.log" 2>&1 &
echo "$!" >"${RUN_DIR}/slam_mapping.pid"

deadline=$((SECONDS + 30))
while [ "${SECONDS}" -lt "${deadline}" ]; do
  nodes="$(ros2 node list 2>/dev/null || true)"
  if printf '%s\n' "${nodes}" | grep -Fxq "/cartographer_node" &&
    printf '%s\n' "${nodes}" | grep -Fxq "/cartographer_occupancy_grid_node"
  then
    echo "Remote map launch started"
    exit 0
  fi
  sleep 0.5
done

echo "Remote map start timed out; see ${LOG_DIR}/slam_mapping.log" >&2
tail -n 80 "${LOG_DIR}/slam_mapping.log" >&2 || true
exit 1
REMOTE

  echo "${REMOTE_HOST}" >"${REMOTE_MARKER}"
}

SKIP_ODOM_PREFLIGHT="${K1_SKIP_ODOM_PREFLIGHT:-0}"
USE_RVIZ="${K1_MAPPING_USE_RVIZ:-false}"

if [ "${SKIP_ODOM_PREFLIGHT}" != "1" ] && ! wait_topic_visible "/odom"; then
  echo "Missing /odom. Start MCU bridge first." >&2
  exit 1
fi

if [ "${REMOTE_COMPUTE}" = "1" ]; then
  start_remote_mapping
  exit 0
fi

if process_exists 'cartographer_node .*k1muse_slam_nav' || process_exists 'k1muse_slam_nav slam.launch.py'; then
  echo "Cartographer mapping already running"
  exit 0
fi

echo "starting local Cartographer mapping with IMU requested by default, use_rviz=${USE_RVIZ}"
launch_args=(use_rviz:="${USE_RVIZ}" enable_imu:=true)
if [ "${SKIP_ODOM_PREFLIGHT}" = "1" ]; then
  launch_args+=(require_odom_precheck:=false)
fi
nohup ros2 launch k1muse_slam_nav slam.launch.py "${launch_args[@]}" >"${LOG_DIR}/slam_mapping.log" 2>&1 &
echo "$!" >"${RUN_DIR}/slam_mapping.pid"

deadline=$((SECONDS + 30))
while [ "${SECONDS}" -lt "${deadline}" ]; do
  if cartographer_ready; then
    if [ -e /dev/myimu ]; then
      echo "Map launch started with IMU"
    else
      echo "WARNING: /dev/myimu missing; map launch fell back without IMU"
      echo "Map launch started"
    fi
    exit 0
  fi
  sleep 0.5
done

echo "Map start timed out before required topics/nodes; see ${LOG_DIR}/slam_mapping.log" >&2
"${SCRIPT_DIR}/k1_stop_map.sh" fast >/dev/null 2>&1 || true
exit 1
