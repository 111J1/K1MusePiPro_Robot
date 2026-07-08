#!/usr/bin/env bash
set -euo pipefail

ADDR="${WAYVNC_ADDR:-0.0.0.0}"
PORT="${WAYVNC_PORT:-5900}"
FPS="${WAYVNC_FPS:-20}"
LOG="${WAYVNC_LOG:-$HOME/wayvnc.log}"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

if [ ! -d "$XDG_RUNTIME_DIR" ]; then
    echo "ERROR: XDG_RUNTIME_DIR does not exist: $XDG_RUNTIME_DIR" >&2
    echo "Login to the graphical desktop first, then run this script." >&2
    exit 1
fi

if [ -z "${WAYLAND_DISPLAY:-}" ]; then
    for socket in "$XDG_RUNTIME_DIR"/wayland-[0-9]; do
        if [ -S "$socket" ]; then
            export WAYLAND_DISPLAY="$(basename "$socket")"
            break
        fi
    done
fi

if [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "ERROR: No Wayland socket found under $XDG_RUNTIME_DIR" >&2
    echo "Login to the graphical desktop first, then run this script." >&2
    exit 1
fi

if pgrep -u "$(id -u)" -x wayvnc >/dev/null 2>&1; then
    echo "wayvnc is already running:"
    pgrep -a -u "$(id -u)" -x wayvnc
    exit 0
fi

if ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq "(^|:)${PORT}$"; then
    echo "ERROR: TCP port ${PORT} is already listening." >&2
    ss -ltnp 2>/dev/null | grep -E "(^|:)${PORT}\b" || true
    exit 1
fi

: > "$LOG"
nohup wayvnc --log-level=info --max-fps="$FPS" --render-cursor "$ADDR" "$PORT" >>"$LOG" 2>&1 &
pid=$!

sleep 1
if kill -0 "$pid" >/dev/null 2>&1; then
    echo "wayvnc started."
    echo "PID: $pid"
    echo "Display: ${WAYLAND_DISPLAY}"
    echo "Listen: ${ADDR}:${PORT}"
    echo "Log: $LOG"
else
    echo "ERROR: wayvnc failed to start. Last log lines:" >&2
    tail -50 "$LOG" >&2 || true
    exit 1
fi
