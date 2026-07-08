#!/usr/bin/env python3
"""
MCU Simulator — simulate STM32G474 via CH340 USB-UART for closed-loop testing.

Protocol: binary frame matching STM32 firmware mdl_control_protocol.h
  A5 5A | SRC TARGET CMD SEQ LEN | PAYLOAD(0~64B) | CRC8

Usage:
  python mcu_sim.py COM3                          # interactive mode
  python mcu_sim.py COM3 --json                    # machine-readable output
  python mcu_sim.py COM3 --interval 50             # 50ms status interval
  python mcu_sim.py COM3 --no-status               # listen only
  python mcu_sim.py COM3 --send-status --json      # one-shot status frame
  python mcu_sim.py --list                         # list COM ports
  python mcu_sim.py                                # auto-detect CH340
"""

import sys
import struct
import time
import json
import atexit
import signal
import argparse
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print(json.dumps({"type": "error",
                      "msg": "pyserial not installed. Run: pip install pyserial"}))
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════════════
# Protocol constants (match STM32 mdl_control_protocol.h)
# ═══════════════════════════════════════════════════════════════════════════════

SOF1, SOF2          = 0xA5, 0x5A
SRC_HOST, SRC_MCU   = 0x02, 0x10

TARGET_MAP = {0x00: "SYSTEM", 0x01: "CHASSIS", 0x02: "ARM", 0x03: "LIFT"}
SRC_MAP    = {0x00: "NONE", 0x01: "BT", 0x02: "HOST", 0x10: "MCU"}

CHASSIS_CMD_MAP = {0x00: "STOP", 0x01: "MOV", 0x02: "ODOM", 0x80: "STATUS"}
CHASSIS_STATUS_SIZE = 32
CHASSIS_MOV_SIZE    = 13
CHASSIS_ODOM_SIZE   = 12

# ═══════════════════════════════════════════════════════════════════════════════
# CRC-8/ATM (poly=0x07, init=0x00)
# ═══════════════════════════════════════════════════════════════════════════════

def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) if (c & 0x80) else (c << 1)
            c &= 0xFF
    return c

# ═══════════════════════════════════════════════════════════════════════════════
# Frame builder / parser
# ═══════════════════════════════════════════════════════════════════════════════

def build_frame(src, target, cmd, seq, payload=b''):
    """Build binary frame: SOF + header + payload + CRC8."""
    header = struct.pack('<BBBBB', src, target, cmd, seq, len(payload))
    body = header + payload
    return b'\xA5\x5A' + body + bytes([crc8(body)])

def parse_frame(buf):
    """Try to parse one frame from buffer.  Returns (src,target,cmd,seq,payload)
    and bytes_consumed, or (None, 0)."""
    if len(buf) < 8:  # minimum: SOF(2)+header(5)+CRC(1)
        return None, 0
    if buf[0] != SOF1 or buf[1] != SOF2:
        return None, 0
    src, target, cmd, seq, plen = struct.unpack('<BBBBB', buf[2:7])
    total = 7 + plen + 1
    if len(buf) < total:
        return None, 0
    payload = buf[7:7 + plen]
    if crc8(buf[2:7 + plen]) != buf[7 + plen]:
        return None, 1  # CRC fail, skip SOF byte to re-sync
    return (src, target, cmd, seq, payload), total

def make_chassis_status(tick_ms, state=1, move_cs=1, block_flags=0,
                        vx=0.1, vy=0.0, omega=0.0,
                        x=0.0, y=0.0, direction=0.0):
    """Build a ChassisStatus frame (SRC=MCU, TARGET=CHASSIS, CMD=0x80)."""
    payload = struct.pack('<IBBBBffffff',
        tick_ms, state, move_cs, block_flags, 0,
        vx, vy, omega, x, y, direction)
    return build_frame(SRC_MCU, 0x01, 0x80, 0, payload)

def fmt_frame(src, target, cmd, seq, payload):
    """Format a parsed frame for human or JSON output."""
    tname = TARGET_MAP.get(target, f"0x{target:02X}")
    if target == 0x01:
        cname = CHASSIS_CMD_MAP.get(cmd, f"0x{cmd:02X}")
    else:
        cname = f"0x{cmd:02X}"
    sname = SRC_MAP.get(src, f"0x{src:02X}")

    info = {"target": tname, "cmd": cname, "src": sname, "seq": seq}

    if target == 0x01:
        if cmd == 0x80 and len(payload) == CHASSIS_STATUS_SIZE:
            v = struct.unpack('<IBBBBffffff', payload)
            info["payload"] = {
                "tick_ms": v[0], "state": v[1], "move_cs": v[2],
                "block_flags": v[3], "vx": round(v[5], 4),
                "vy": round(v[6], 4), "omega": round(v[7], 4),
                "x": round(v[8], 4), "y": round(v[9], 4),
                "direction": round(v[10], 4)}
        elif cmd == 0x01 and len(payload) == CHASSIS_MOV_SIZE:
            mc, d, vel, om = struct.unpack('<Bfff', payload)
            info["payload"] = {"move_cs": mc, "direction": round(d, 4),
                               "v": round(vel, 4), "omega": round(om, 4)}
        elif cmd == 0x02 and len(payload) == CHASSIS_ODOM_SIZE:
            d, px, py = struct.unpack('<fff', payload)
            info["payload"] = {"direction": round(d, 4),
                               "x": round(px, 4), "y": round(py, 4)}
        elif len(payload) == 0:
            info["payload"] = {}
        else:
            info["payload"] = {"raw_len": len(payload),
                               "hex": payload.hex().upper()}

    return info

# ═══════════════════════════════════════════════════════════════════════════════
# CH340 auto-detection
# ═══════════════════════════════════════════════════════════════════════════════

def list_ports():
    """List all available COM ports."""
    ports = serial.tools.list_ports.comports()
    results = []
    for p in ports:
        results.append({
            "device": p.device,
            "description": p.description,
            "hwid": p.hwid,
            "is_ch340": "CH340" in p.description.upper() or
                        "1A86" in p.hwid.upper()})
    return results

def find_ch340():
    """Find the first CH340 COM port, or return None."""
    for p in serial.tools.list_ports.comports():
        hwid = (p.hwid or "").upper()
        desc = (p.description or "").upper()
        if "CH340" in desc or "1A86" in hwid or "7523" in hwid:
            return p.device
    return None

# ═══════════════════════════════════════════════════════════════════════════════
# Simulator
# ═══════════════════════════════════════════════════════════════════════════════

class McuSimulator:
    def __init__(self, port, baudrate=115200, json_out=False):
        self.port_name = port
        self.json_out = json_out
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.ser.dtr = False
        self.ser.rts = False
        self.stream_buf = b''
        self.tick = 0
        self.start_time = time.time()
        self._closed = False
        atexit.register(self.close)

    def close(self):
        if self._closed:
            return
        self._closed = True
        atexit.unregister(self.close)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def _emit(self, evt):
        if self.json_out:
            print(json.dumps(evt, ensure_ascii=False), flush=True)
        else:
            t = evt.get("type", "?")
            tag = {"tx": "[TX]", "rx": "[RX]", "info": "[--]",
                   "error": "[!!]"}.get(t, "[??]")
            ts = evt.get("time", 0)
            ts_str = f"{ts:10.3f}" if ts else ""
            if t == "tx":
                fi = evt.get("frame", {})
                pl = fi.get("payload", {})
                extra = ""
                if "vx" in pl:
                    extra = f" v={pl['vx']:.2f},{pl['vy']:.2f} ω={pl['omega']:.2f}"
                print(f"{tag} {ts_str} {fi['target']}/{fi['cmd']} seq={fi['seq']}"
                      f" len={evt.get('len',0)}{extra}")
            elif t == "rx":
                fi = evt.get("frame", {})
                pl = fi.get("payload", {})
                extra = ""
                if "v" in pl:
                    extra = f" v={pl['v']:.2f} ω={pl['omega']:.2f}"
                elif "direction" in pl and "x" in pl:
                    extra = f" dir={pl['direction']:.2f} x={pl['x']:.2f},y={pl['y']:.2f}"
                print(f"{tag} {ts_str} {fi['src']}>{fi['target']}/{fi['cmd']}"
                      f" seq={fi['seq']}{extra}")
            elif t == "info":
                print(f"{tag} {evt['msg']}")
            elif t == "error":
                print(f"{tag} {evt['msg']}")

    def write_frame(self, data):
        self.ser.write(data)
        # parse for logging
        f, _ = parse_frame(data)
        if f:
            src, target, cmd, seq, payload = f
            info = fmt_frame(src, target, cmd, seq, payload)
            self._emit({
                "type": "tx", "time": time.time() - self.start_time,
                "len": len(data), "hex": data.hex().upper(),
                "frame": info})

    def send_status(self):
        self.tick += 10  # 10ms increments like real MCU
        elapsed = int((time.time() - self.start_time) * 1000)
        frame = make_chassis_status(
            tick_ms=elapsed,
            state=1, move_cs=1, block_flags=0,
            vx=0.1 + 0.01 * (self.tick % 100) / 100,
            vy=0.0, omega=0.05,
            x=self.tick * 0.001, y=0.0, direction=self.tick * 0.0005)
        self.write_frame(frame)

    def poll_rx(self):
        """Read available bytes, parse frames, return list."""
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except serial.SerialException:
            return []
        if not data:
            return []

        self.stream_buf += data
        frames = []
        while True:
            f, consumed = parse_frame(self.stream_buf)
            if f is None:
                if consumed:
                    self.stream_buf = self.stream_buf[consumed:]
                    continue
                if len(self.stream_buf) > 256:
                    self.stream_buf = self.stream_buf[-128:]
                break
            frames.append(f)
            self.stream_buf = self.stream_buf[consumed:]
        return frames

    def log_rx_frames(self, frames):
        for src, target, cmd, seq, payload in frames:
            info = fmt_frame(src, target, cmd, seq, payload)
            self._emit({
                "type": "rx", "time": time.time() - self.start_time,
                "frame": info})

    def run(self, interval_ms=100):
        """Main loop: send status periodically, print received frames."""
        status_interval = interval_ms / 1000.0
        last_status = 0

        self._emit({"type": "info", "time": 0,
                     "msg": f"MCU Simulator on {self.port_name}, "
                            f"status interval={interval_ms}ms"})

        while True:
            try:
                now = time.time()

                # send status frame
                if interval_ms > 0 and now - last_status >= status_interval:
                    self.send_status()
                    last_status = now

                # receive frames
                frames = self.poll_rx()
                if frames:
                    self.log_rx_frames(frames)

            except KeyboardInterrupt:
                self._emit({"type": "info",
                            "msg": "stopped by user"})
                break
            except serial.SerialException as e:
                self._emit({"type": "error",
                            "msg": f"serial error: {e}"})
                break

        self.close()

# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="MCU Simulator — simulate STM32 via CH340 USB-UART")
    parser.add_argument("port", nargs="?",
                        help="COM port (e.g. COM3). Auto-detects CH340 if omitted.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--interval", type=int, default=100,
                        help="Status send interval in ms (0=don't send, default 100)")
    parser.add_argument("--json", action="store_true",
                        help="Machine-readable JSON output")
    parser.add_argument("--list", action="store_true",
                        help="List available COM ports and exit")
    parser.add_argument("--send-status", action="store_true",
                        help="Send one status frame and exit")
    parser.add_argument("--no-status", action="store_true",
                        help="Don't send periodic status (listen only)")
    args = parser.parse_args()

    if args.list:
        ports = list_ports()
        if args.json:
            print(json.dumps(ports, indent=2, ensure_ascii=False))
        else:
            if not ports:
                print("No COM ports found.")
            for p in ports:
                tag = " [CH340]" if p["is_ch340"] else ""
                print(f"{p['device']:8s} {p['description']}{tag}")
        return

    if not args.port:
        auto = find_ch340()
        if not auto:
            ports = list_ports()
            if not ports:
                print("ERROR: No COM ports found. Is CH340 plugged in?",
                      file=sys.stderr)
            else:
                print("ERROR: Could not auto-detect CH340. Available ports:",
                      file=sys.stderr)
                for p in ports:
                    print(f"  {p['device']} - {p['description']}", file=sys.stderr)
            sys.exit(1)
        args.port = auto
        if not args.json:
            print(f"[--] Auto-detected CH340: {args.port}")

    if args.no_status:
        args.interval = 0

    sim = McuSimulator(args.port, args.baud, json_out=args.json)
    try:
        if args.send_status:
            sim.send_status()
            return

        sim.run(interval_ms=args.interval)
    finally:
        sim.close()

if __name__ == "__main__":
    main()
