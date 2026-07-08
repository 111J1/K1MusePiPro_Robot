#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-manual}"
case "${MODE}" in
  manual) PREFIX="manual" ;;
  stop) PREFIX="stop" ;;
  auto) PREFIX="auto" ;;
  *) echo "unknown save mode: ${MODE}" >&2; exit 2 ;;
esac

STATE_DIR="/home/bianbu/.ros/k1muse_slam_nav"
LOG_DIR="${STATE_DIR}/logs"
MAP_DIR="/home/bianbu/k1muse_slam_ros/src/k1muse_slam_nav/maps"
mkdir -p "${LOG_DIR}" "${MAP_DIR}"

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

STAMP="$(date +%Y%m%d_%H%M%S)"
MAP_BASE="${MAP_DIR}/${PREFIX}_${STAMP}"
LOG_FILE="${LOG_DIR}/save_${PREFIX}_${STAMP}.log"

REMOTE_COMPUTE="${K1_REMOTE_COMPUTE:-1}"
REMOTE_HOST="${K1_REMOTE_SLAM_HOST:-user@10.10.10.1}"
REMOTE_COMMS_WS="${K1_REMOTE_COMMS_WS:-/home/user/k1muse_communicate_ros}"
REMOTE_WS="${K1_REMOTE_SLAM_WS:-/home/user/k1muse_slam_ros}"
REMOTE_MAP_DIR="${K1_REMOTE_MAP_DIR:-${REMOTE_WS}/src/k1muse_slam_nav/maps}"

if [ "${REMOTE_COMPUTE}" = "1" ]; then
  REMOTE_MAP_BASE="${REMOTE_MAP_DIR}/${PREFIX}_${STAMP}"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_HOST}" \
    "REMOTE_COMMS_WS='${REMOTE_COMMS_WS}' REMOTE_WS='${REMOTE_WS}' REMOTE_MAP_BASE='${REMOTE_MAP_BASE}' bash -s" >"${LOG_FILE}" 2>&1 <<'REMOTE'
set -euo pipefail
mkdir -p "$(dirname "${REMOTE_MAP_BASE}")"
set +u
source /opt/ros/humble/setup.bash
source "${REMOTE_COMMS_WS}/install/setup.bash"
source "${REMOTE_WS}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
if [ -z "${CYCLONEDDS_URI:-}" ] && [ -f "${HOME}/.ros/cyclonedds_eth.xml" ]; then
  export CYCLONEDDS_URI="file://${HOME}/.ros/cyclonedds_eth.xml"
fi

timeout 20 ros2 topic echo --once /map --field info
echo "saving occupancy map to ${REMOTE_MAP_BASE}"
timeout 60 ros2 run nav2_map_server map_saver_cli -f "${REMOTE_MAP_BASE}"
echo "saving Cartographer state to ${REMOTE_MAP_BASE}.pbstream"
timeout 60 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${REMOTE_MAP_BASE}.pbstream', include_unfinished_submaps: true}"
REMOTE

  scp -q "${REMOTE_HOST}:${REMOTE_MAP_BASE}.yaml" "${MAP_BASE}.yaml" >>"${LOG_FILE}" 2>&1
  scp -q "${REMOTE_HOST}:${REMOTE_MAP_BASE}.pgm" "${MAP_BASE}.pgm" >>"${LOG_FILE}" 2>&1
  scp -q "${REMOTE_HOST}:${REMOTE_MAP_BASE}.pbstream" "${MAP_BASE}.pbstream" >>"${LOG_FILE}" 2>&1

  echo "MAP_BASE=${MAP_BASE}"
  echo "Map saved: ${MAP_BASE}"
  exit 0
fi

if ! timeout 20 ros2 topic echo --once /map --field info >"${LOG_FILE}" 2>&1; then
  echo "No /map frame available; see ${LOG_FILE}" >&2
  exit 1
fi

echo "saving occupancy map to ${MAP_BASE}" | tee -a "${LOG_FILE}"
timeout 60 ros2 run nav2_map_server map_saver_cli -f "${MAP_BASE}" >>"${LOG_FILE}" 2>&1

echo "saving Cartographer state to ${MAP_BASE}.pbstream" | tee -a "${LOG_FILE}"
timeout 60 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${MAP_BASE}.pbstream', include_unfinished_submaps: true}" >>"${LOG_FILE}" 2>&1

echo "MAP_BASE=${MAP_BASE}"
echo "Map saved: ${MAP_BASE}"
