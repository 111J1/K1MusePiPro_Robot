#!/usr/bin/env python3
"""Low-resource Bluetooth entrypoint for the Android K1 app.

The supervisor is intentionally smaller than mobile_bridge_node. It only accepts
bridge lifecycle commands, then starts the full ROS bridge and waits until it
exits before returning to supervisor mode.
"""

import grp
import os
import select
import signal
import struct
import subprocess
import termios
import threading
import time
import zlib


DEVICE = "/dev/rfcomm0"
CHANNEL = "1"
DEVICE_GROUP = "bianbu"
MAGIC = b"K1MB"
VERSION = 1
HELLO = 0x01
BRIDGE_CONTROL = 0x48
BRIDGE_STATUS = 0x49
START_BRIDGE = 1
STOP_BRIDGE = 2
QUERY_BRIDGE = 3
STATE_SUPERVISOR = 0
STATE_STARTING = 1
STATE_ONLINE = 2
STATE_STOPPING = 3
STATE_ERROR = 4


def append_string(value):
    raw = value.encode("utf-8", errors="replace")[:65535]
    return struct.pack("<H", len(raw)) + raw


def frame(msg_type, seq, payload=b""):
    header = struct.pack(
        "<4sBBHIII",
        MAGIC,
        VERSION,
        msg_type,
        0,
        seq & 0xFFFFFFFF,
        len(payload),
        zlib.crc32(payload) & 0xFFFFFFFF,
    )
    return header + payload


def hello_payload():
    return append_string("k1muse_mobile_supervisor") + bytes([1])


def bridge_status_payload(state, command, success, message):
    return (
        struct.pack("<IBBB", int(time.time() * 1000) & 0xFFFFFFFF, state, command, 1 if success else 0)
        + append_string(message)
    )


def parse_frames(buffer):
    frames = []
    while True:
        start = buffer.find(MAGIC)
        if start < 0:
            return bytearray(), frames
        if start:
            del buffer[:start]
        if len(buffer) < 20:
            return buffer, frames
        magic, version, msg_type, _flags, seq, payload_len, expected_crc = struct.unpack(
            "<4sBBHIII", buffer[:20]
        )
        if magic != MAGIC or version != VERSION or payload_len > 4 * 1024 * 1024:
            del buffer[0]
            continue
        total = 20 + payload_len
        if len(buffer) < total:
            return buffer, frames
        payload = bytes(buffer[20:total])
        del buffer[:total]
        if zlib.crc32(payload) & 0xFFFFFFFF != expected_crc:
            continue
        frames.append((msg_type, seq, payload))


def terminate_process(proc, timeout=3.0):
    if proc is None or proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout)


def terminate_process_group(proc, timeout=3.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    except Exception as exc:
        print(f"failed to terminate process group {proc.pid}: {exc}", flush=True)
        terminate_process(proc, timeout=timeout)
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=timeout)


def stop_ros2_daemon():
    try:
        output = subprocess.check_output(
            ["pgrep", "-f", "ros2cli.daemon.daemonize"],
            stderr=subprocess.DEVNULL,
            timeout=2,
        ).decode()
    except Exception:
        return
    own_pid = os.getpid()
    for raw_pid in output.split():
        try:
            pid = int(raw_pid)
        except ValueError:
            continue
        if pid == own_pid:
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception as exc:
            print(f"failed to kill ros2 daemon pid {pid}: {exc}", flush=True)


class MobileSupervisor:
    def __init__(self):
        self.seq = 1
        self.rfcomm_proc = None
        self.bridge_proc = None
        self.rfcomm_lock = threading.Lock()
        self.permission_running = False
        self.permission_thread = None
        self.rfcomm_restart_running = False
        self.rfcomm_restart_thread = None

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return self.seq

    def write_frame(self, fd, msg_type, payload):
        os.write(fd, frame(msg_type, self.next_seq(), payload))

    def set_device_permissions(self):
        if not os.path.exists(DEVICE):
            return
        try:
            gid = grp.getgrnam(DEVICE_GROUP).gr_gid
            os.chown(DEVICE, 0, gid)
            os.chmod(DEVICE, 0o660)
        except Exception as exc:
            print(f"failed to set {DEVICE} permissions: {exc}", flush=True)

    def start_permission_watcher(self):
        self.permission_running = True

        def watch():
            while self.permission_running:
                self.set_device_permissions()
                time.sleep(0.2)

        self.permission_thread = threading.Thread(target=watch, daemon=True)
        self.permission_thread.start()

    def stop_permission_watcher(self):
        self.permission_running = False
        if self.permission_thread is not None:
            self.permission_thread.join(timeout=1.0)
        self.permission_thread = None

    def start_rfcomm(self):
        self.start_permission_watcher()
        self.spawn_rfcomm_listener()

    def stop_rfcomm(self):
        self.stop_permission_watcher()
        with self.rfcomm_lock:
            terminate_process(self.rfcomm_proc)
            self.rfcomm_proc = None
        subprocess.run(["rfcomm", "release", DEVICE], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def spawn_rfcomm_listener(self):
        with self.rfcomm_lock:
            if self.rfcomm_proc is not None and self.rfcomm_proc.poll() is None:
                return
            subprocess.run(["rfcomm", "release", DEVICE], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.2)
            subprocess.run(["sdptool", "add", f"--channel={CHANNEL}", "SP"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.rfcomm_proc = subprocess.Popen(["rfcomm", "listen", DEVICE, CHANNEL])

    def start_rfcomm_restart_loop(self):
        self.start_permission_watcher()
        self.rfcomm_restart_running = True

        def watch():
            while self.rfcomm_restart_running:
                self.spawn_rfcomm_listener()
                time.sleep(0.5)

        self.rfcomm_restart_thread = threading.Thread(target=watch, daemon=True)
        self.rfcomm_restart_thread.start()

    def stop_rfcomm_restart_loop(self):
        self.rfcomm_restart_running = False
        if self.rfcomm_restart_thread is not None:
            self.rfcomm_restart_thread.join(timeout=1.0)
        self.rfcomm_restart_thread = None
        self.stop_rfcomm()

    def open_device(self, timeout=None):
        deadline = None if timeout is None else time.time() + timeout
        while deadline is None or time.time() < deadline:
            self.set_device_permissions()
            try:
                fd = os.open(DEVICE, os.O_RDWR | os.O_NOCTTY)
                tty = termios.tcgetattr(fd)
                tty[0] = 0
                tty[1] = 0
                tty[2] = tty[2] | termios.CLOCAL | termios.CREAD
                tty[3] = 0
                tty[6][termios.VMIN] = 1
                tty[6][termios.VTIME] = 0
                termios.tcsetattr(fd, termios.TCSANOW, tty)
                return fd
            except OSError:
                time.sleep(0.2)
        raise TimeoutError(f"timed out waiting for {DEVICE}")

    def serve_supervisor_connection(self):
        self.start_rfcomm()
        fd = None
        try:
            print("supervisor waiting for phone connection", flush=True)
            fd = self.open_device()
            print("supervisor connected", flush=True)
            self.write_frame(fd, HELLO, hello_payload())
            self.write_frame(
                fd,
                BRIDGE_STATUS,
                bridge_status_payload(STATE_SUPERVISOR, QUERY_BRIDGE, True, "supervisor online"),
            )
            rx = bytearray()
            while True:
                ready, _, _ = select.select([fd], [], [], 0.2)
                if not ready:
                    continue
                chunk = os.read(fd, 1024)
                if not chunk:
                    return "disconnect"
                rx.extend(chunk)
                rx, frames = parse_frames(rx)
                for msg_type, _seq, payload in frames:
                    if msg_type != BRIDGE_CONTROL or len(payload) < 5:
                        continue
                    command = payload[4]
                    if command == START_BRIDGE:
                        self.write_frame(
                            fd,
                            BRIDGE_STATUS,
                            bridge_status_payload(STATE_STARTING, START_BRIDGE, True, "starting mobile bridge"),
                        )
                        return "start_bridge"
                    if command == QUERY_BRIDGE:
                        self.write_frame(
                            fd,
                            BRIDGE_STATUS,
                            bridge_status_payload(STATE_SUPERVISOR, QUERY_BRIDGE, True, "supervisor online"),
                        )
                    if command == STOP_BRIDGE:
                        self.write_frame(
                            fd,
                            BRIDGE_STATUS,
                            bridge_status_payload(STATE_SUPERVISOR, STOP_BRIDGE, True, "bridge is not running"),
                        )
        finally:
            if fd is not None:
                os.close(fd)
            self.stop_rfcomm()

    def _rfcomm_is_connected(self):
        """Return True if a phone is currently connected via RFCOMM."""
        try:
            output = subprocess.check_output(["rfcomm"], stderr=subprocess.DEVNULL, timeout=3).decode()
            for line in output.splitlines():
                if line.startswith("rfcomm0:") and " closed " not in line:
                    return True
            return False
        except Exception:
            return False

    def _bridge_idle_watcher(self, idle_timeout=45):
        """Watch for bridge idle and terminate if no phone connection for too long.

        The watcher gives the bridge a grace period to establish an initial
        connection, then periodically checks whether a phone is still connected.
        If the bridge runs without a phone connection for ``idle_timeout`` seconds,
        the bridge process is terminated so the supervisor can recover and serve
        new connections.
        """
        start = time.time()
        last_connected = start
        while self.bridge_proc is not None and self.bridge_proc.poll() is None:
            time.sleep(3)
            elapsed = time.time() - start
            # Give the bridge a startup grace period before treating no phone as idle.
            if elapsed < 20:
                continue
            if self._rfcomm_is_connected():
                last_connected = time.time()
            elif time.time() - last_connected > idle_timeout:
                print(
                    f"mobile bridge idle for {idle_timeout}s; terminating",
                    flush=True,
                )
                terminate_process_group(self.bridge_proc, timeout=5.0)
                return

    def start_bridge_session(self):
        print("starting mobile bridge session", flush=True)
        self.start_rfcomm_restart_loop()
        command = (
            "export ROS_DISABLE_DAEMON=true && "
            "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} && "
            "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} && "
            "export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0} && "
            "if [ -z \"${CYCLONEDDS_URI:-}\" ] && [ -f \"$HOME/.ros/cyclonedds_end.xml\" ]; "
            "then export CYCLONEDDS_URI=\"file://$HOME/.ros/cyclonedds_end.xml\"; fi && "
            "source /opt/ros/humble/setup.bash && "
            "source /home/bianbu/k1muse_communicate_ros/install/setup.bash && "
            "ros2 run k1muse_mobile_bridge mobile_bridge_node --ros-args "
            "--params-file /home/bianbu/k1muse_communicate_ros/install/k1muse_mobile_bridge/share/k1muse_mobile_bridge/config/mobile_bridge.yaml"
        )
        self.bridge_proc = subprocess.Popen(
            ["runuser", "-u", "bianbu", "--", "bash", "-lc", command],
            start_new_session=True,
        )
        idle_thread = threading.Thread(target=self._bridge_idle_watcher, args=(45,), daemon=True)
        idle_thread.start()
        try:
            self.bridge_proc.wait()
            print(f"mobile bridge exited rc={self.bridge_proc.returncode}", flush=True)
        finally:
            self.bridge_proc = None
            self.stop_rfcomm_restart_loop()
            stop_ros2_daemon()

    def run(self):
        def handle_term(_sig, _frame):
            raise SystemExit(0)

        signal.signal(signal.SIGTERM, handle_term)
        try:
            while True:
                action = self.serve_supervisor_connection()
                if action == "start_bridge":
                    self.start_bridge_session()
                time.sleep(0.5)
        finally:
            terminate_process_group(self.bridge_proc, timeout=5.0)
            self.stop_rfcomm_restart_loop()
            self.stop_rfcomm()
            stop_ros2_daemon()


if __name__ == "__main__":
    MobileSupervisor().run()
