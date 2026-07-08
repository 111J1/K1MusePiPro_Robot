#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="/home/bianbu/.ros/k1muse_slam_nav"
RUN_DIR="${STATE_DIR}/run"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

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

topic_visible() {
  if [ "$1" = "/scan" ] && grep -q "scan self-filter" "${LOG_DIR}/sensors.log" 2>/dev/null; then
    return 0
  fi
  timeout 3 ros2 topic echo --once "$1" >/dev/null 2>&1 ||
    timeout 2 ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

wait_topic_visible() {
  local topic="$1"
  local deadline=$((SECONDS + 45))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if topic_visible "${topic}"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

if process_exists 'k1muse_slam_nav _sensors\.launch\.py'; then
  echo "K1 sensors already running"
  exit 0
fi

: >"${LOG_DIR}/sensors.log"
nohup ros2 launch k1muse_slam_nav _sensors.launch.py \
  enable_imu:="${K1_SENSORS_ENABLE_IMU:-true}" \
  require_odom_precheck:="${K1_SENSORS_REQUIRE_ODOM_PREFLIGHT:-false}" \
  >"${LOG_DIR}/sensors.log" 2>&1 &
echo "$!" >"${RUN_DIR}/sensors.pid"

if wait_topic_visible "/scan"; then
  echo "K1 sensors started"
  exit 0
fi

echo "K1 sensors start timed out; see ${LOG_DIR}/sensors.log" >&2
exit 1
