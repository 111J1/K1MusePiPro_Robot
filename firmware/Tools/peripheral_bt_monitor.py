#!/usr/bin/env python3
"""Monitor peripheral telemetry frames through the Bluetooth serial port."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - depends on PC environment
    serial = None
    list_ports = None


SOF = bytes((0xA5, 0x5A))
CTRL_PROTOCOL_MAX_PAYLOAD = 64

CTRL_SRC_MCU = 0x10
CTRL_TARGET_PERIPHERAL = 0x04
CTRL_PERIPH_RPT_STATUS = 0x80

POWER_STATE_NAMES = {
    0x00: "NORMAL",
    0x01: "LOW",
    0x02: "CRITICAL",
    0x03: "FAULT",
}


@dataclass
class DecodedFrame:
    src: int
    target: int
    cmd: int
    seq: int
    payload: bytes
    raw: bytes


@dataclass
class PeripheralStatus:
    tick_ms: int
    gas_detected: int
    power_state: int
    power_mv: int
    temperature_centi_c: int
    humidity_centi_pct: int

    @property
    def temperature_c(self) -> float:
        return self.temperature_centi_c / 100.0

    @property
    def humidity_pct(self) -> float:
        return self.humidity_centi_pct / 100.0


def crc8_atm(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_crc_count = 0

    def feed(self, data: bytes) -> list[DecodedFrame]:
        self.buffer.extend(data)
        frames: list[DecodedFrame] = []

        while True:
            sof_index = self.buffer.find(SOF)
            if sof_index < 0:
                self.buffer.clear()
                return frames
            if sof_index > 0:
                del self.buffer[:sof_index]
            if len(self.buffer) < 8:
                return frames

            src, target, cmd, seq, length = self.buffer[2:7]
            if length > CTRL_PROTOCOL_MAX_PAYLOAD:
                del self.buffer[0]
                continue

            frame_len = 2 + 5 + length + 1
            if len(self.buffer) < frame_len:
                return frames

            raw = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]

            body = raw[2:-1]
            if crc8_atm(body) != raw[-1]:
                self.bad_crc_count += 1
                continue

            frames.append(DecodedFrame(src, target, cmd, seq, raw[7:-1], raw))


def list_serial_ports() -> int:
    if list_ports is None:
        print("pyserial is not installed. Run: python -m pip install pyserial")
        return 1

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 0

    for port in ports:
        print(f"{port.device:>8}  {port.description}")
    return 0


def decode_peripheral_status(frame: DecodedFrame) -> PeripheralStatus:
    if len(frame.payload) != 12:
        raise ValueError(f"bad peripheral payload length: {len(frame.payload)}")
    return PeripheralStatus(*struct.unpack("<IBBHhH", frame.payload))


def build_warnings(status: PeripheralStatus, args: argparse.Namespace) -> list[str]:
    warnings: list[str] = []

    if args.temp_min is not None and status.temperature_c < args.temp_min:
        warnings.append("TEMP_LOW")
    if args.temp_max is not None and status.temperature_c > args.temp_max:
        warnings.append("TEMP_HIGH")
    if args.rh_min is not None and status.humidity_pct < args.rh_min:
        warnings.append("RH_LOW")
    if args.rh_max is not None and status.humidity_pct > args.rh_max:
        warnings.append("RH_HIGH")

    return warnings


def print_peripheral_status(
    frame: DecodedFrame,
    status: PeripheralStatus,
    host_time: float,
    last_tick_ms: int | None,
    last_host_time: float | None,
    warnings: list[str],
    show_raw: bool,
) -> None:
    state_name = POWER_STATE_NAMES.get(status.power_state, f"UNKNOWN({status.power_state})")

    tick_gap = "-"
    if last_tick_ms is not None:
        tick_gap = str((status.tick_ms - last_tick_ms) & 0xFFFFFFFF)

    host_gap = "-"
    if last_host_time is not None:
        host_gap = f"{(host_time - last_host_time) * 1000.0:.1f}"

    warn_text = "OK" if not warnings else ",".join(warnings)
    line = (
        f"{time.strftime('%H:%M:%S')} "
        f"seq={frame.seq:03d} tick={status.tick_ms:10d}ms "
        f"dt_tick={tick_gap:>4}ms dt_host={host_gap:>6}ms "
        f"gas={status.gas_detected} power={state_name:<8} power_mv={status.power_mv:5d} "
        f"temp={status.temperature_c:6.2f}C rh={status.humidity_pct:6.2f}% {warn_text}"
    )
    if show_raw:
        line += f" raw=[{hex_bytes(frame.raw)}]"
    print(line)


def write_csv_header(writer: csv.writer) -> None:
    writer.writerow(
        [
            "host_time",
            "elapsed_s",
            "seq",
            "tick_ms",
            "dt_tick_ms",
            "dt_host_ms",
            "gas_detected",
            "power_state",
            "power_state_name",
            "power_mv",
            "temperature_c",
            "humidity_pct",
            "warnings",
            "raw",
        ]
    )


def write_csv_row(
    writer: csv.writer,
    start_time: float,
    host_time: float,
    frame: DecodedFrame,
    status: PeripheralStatus,
    last_tick_ms: int | None,
    last_host_time: float | None,
    warnings: list[str],
) -> None:
    dt_tick = ""
    if last_tick_ms is not None:
        dt_tick = str((status.tick_ms - last_tick_ms) & 0xFFFFFFFF)

    dt_host = ""
    if last_host_time is not None:
        dt_host = f"{(host_time - last_host_time) * 1000.0:.1f}"

    writer.writerow(
        [
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            f"{host_time - start_time:.3f}",
            frame.seq,
            status.tick_ms,
            dt_tick,
            dt_host,
            status.gas_detected,
            status.power_state,
            POWER_STATE_NAMES.get(status.power_state, f"UNKNOWN({status.power_state})"),
            status.power_mv,
            f"{status.temperature_c:.2f}",
            f"{status.humidity_pct:.2f}",
            ";".join(warnings),
            hex_bytes(frame.raw),
        ]
    )


def monitor(args: argparse.Namespace) -> int:
    if serial is None:
        print("pyserial is not installed. Run: python -m pip install pyserial")
        return 1

    parser = FrameParser()
    start_time = time.monotonic()
    last_tick_ms: int | None = None
    last_host_time: float | None = None
    peripheral_count = 0
    other_count = 0
    csv_file = None
    csv_writer = None

    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        write_csv_header(csv_writer)

    print(f"Opening {args.port} at {args.baudrate} baud...")
    try:
        with serial.Serial(args.port, args.baudrate, timeout=args.timeout) as ser:
            ser.reset_input_buffer()
            if args.csv:
                print(f"CSV logging to {args.csv}")
            print("Listening for peripheral status frames. Press Ctrl+C to stop.")

            while True:
                if args.duration > 0 and (time.monotonic() - start_time) >= args.duration:
                    break

                data = ser.read(args.chunk_size)
                if not data:
                    continue

                for frame in parser.feed(data):
                    host_time = time.monotonic()
                    is_peripheral = (
                        frame.src == CTRL_SRC_MCU
                        and frame.target == CTRL_TARGET_PERIPHERAL
                        and frame.cmd == CTRL_PERIPH_RPT_STATUS
                    )
                    if is_peripheral:
                        try:
                            status = decode_peripheral_status(frame)
                            warnings = build_warnings(status, args)
                            print_peripheral_status(
                                frame,
                                status,
                                host_time,
                                last_tick_ms,
                                last_host_time,
                                warnings,
                                args.raw,
                            )
                            if csv_writer is not None:
                                write_csv_row(
                                    csv_writer,
                                    start_time,
                                    host_time,
                                    frame,
                                    status,
                                    last_tick_ms,
                                    last_host_time,
                                    warnings,
                                )
                                csv_file.flush()
                            last_tick_ms = status.tick_ms
                            last_host_time = host_time
                            peripheral_count += 1
                        except ValueError as exc:
                            print(f"PERIPHERAL decode error: {exc} raw=[{hex_bytes(frame.raw)}]")
                    elif args.all:
                        other_count += 1
                        print(
                            f"{time.strftime('%H:%M:%S')} "
                            f"other src=0x{frame.src:02X} target=0x{frame.target:02X} "
                            f"cmd=0x{frame.cmd:02X} seq={frame.seq:03d} "
                            f"len={len(frame.payload)} raw=[{hex_bytes(frame.raw)}]"
                        )
    finally:
        if csv_file is not None:
            csv_file.close()

    print(
        "Summary: "
        f"peripheral={peripheral_count}, other={other_count}, bad_crc={parser.bad_crc_count}"
    )
    return 0 if peripheral_count > 0 else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Monitor MCU peripheral status telemetry from a Bluetooth COM port"
    )
    parser.add_argument("--port", default="COM13", help="Bluetooth serial port, default: COM13")
    parser.add_argument("--baudrate", type=int, default=115200, help="baudrate, default: 115200")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run, 0 means forever")
    parser.add_argument("--timeout", type=float, default=0.05, help="serial read timeout in seconds")
    parser.add_argument("--chunk-size", type=int, default=128, help="serial read chunk size")
    parser.add_argument("--raw", action="store_true", help="print raw frame bytes")
    parser.add_argument("--all", action="store_true", help="also print non-peripheral frames")
    parser.add_argument("--csv", help="write peripheral status rows to a CSV file")
    parser.add_argument("--temp-min", type=float, help="warn when temperature is below this Celsius value")
    parser.add_argument("--temp-max", type=float, help="warn when temperature is above this Celsius value")
    parser.add_argument("--rh-min", type=float, help="warn when relative humidity is below this percent value")
    parser.add_argument("--rh-max", type=float, help="warn when relative humidity is above this percent value")
    parser.add_argument("--list-ports", action="store_true", help="list available serial ports and exit")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.list_ports:
        return list_serial_ports()
    try:
        return monitor(args)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
