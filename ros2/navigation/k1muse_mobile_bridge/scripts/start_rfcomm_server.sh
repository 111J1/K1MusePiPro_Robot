#!/usr/bin/env bash
set -euo pipefail

CHANNEL="${1:-1}"
DEVICE="${2:-/dev/rfcomm0}"
DEVICE_GROUP="$(id -gn)"

if ! command -v rfcomm >/dev/null 2>&1; then
  echo "rfcomm command not found. Install bluez first." >&2
  exit 1
fi

start_permission_watcher() {
  (
    while true; do
      if [ -e "${DEVICE}" ]; then
        sudo chgrp "${DEVICE_GROUP}" "${DEVICE}" >/dev/null 2>&1 || true
        sudo chmod 660 "${DEVICE}" >/dev/null 2>&1 || true
      fi
      sleep 0.2
    done
  ) &
  PERMISSION_WATCHER_PID=$!
}

if command -v sdptool >/dev/null 2>&1; then
  sudo sdptool add --channel="${CHANNEL}" SP >/dev/null 2>&1 || \
    echo "Warning: failed to register Serial Port SDP service; Android will use channel fallback." >&2
else
  echo "Warning: sdptool not found; Android will use channel fallback." >&2
fi

while true; do
  sudo rfcomm release "${DEVICE}" >/dev/null 2>&1 || true
  start_permission_watcher
  echo "Waiting for phone RFCOMM connection on channel ${CHANNEL}; device will be ${DEVICE}"
  sudo rfcomm listen "${DEVICE}" "${CHANNEL}" || true
  kill "${PERMISSION_WATCHER_PID}" >/dev/null 2>&1 || true
  wait "${PERMISSION_WATCHER_PID}" 2>/dev/null || true
  sleep 0.5
done
