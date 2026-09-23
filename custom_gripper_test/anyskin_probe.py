"""Probe serial ports and report which ones look like AnySkin streams."""

from __future__ import annotations

import argparse
import glob
import struct
import time

import serial


def parse_ports(text: str) -> list[str]:
    if text:
        return [item.strip() for item in text.split(",") if item.strip()]
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def count_burst_packets(data: bytes, num_mags: int) -> int:
    msg_floats = 4 * num_mags
    msg_length = 4 * msg_floats + 2
    count = 0
    start = 0
    while True:
        end = data.find(b"\r\n", start)
        if end < 0:
            return count
        packet_start = end + 2 - msg_length
        if packet_start >= 0:
            packet = data[packet_start : end + 2]
            try:
                struct.unpack("@{}fcc".format(msg_floats), packet)
                count += 1
            except struct.error:
                pass
        start = end + 2


def read_port(
    port: str,
    baudrate: int,
    seconds: float,
    num_mags: int,
    reset_wait_sec: float,
) -> None:
    print(f"\n{port}: opening @ {baudrate}")
    try:
        with serial.Serial(port=port, baudrate=baudrate, timeout=0.05) as ser:
            ser.dtr = True
            ser.rts = True
            if reset_wait_sec > 0:
                time.sleep(reset_wait_sec)
            ser.reset_input_buffer()
            deadline = time.monotonic() + seconds
            data = bytearray()
            while time.monotonic() < deadline:
                chunk = ser.read(max(1, ser.in_waiting))
                if chunk:
                    data.extend(chunk)
                time.sleep(0.01)
    except Exception as exc:
        print(f"{port}: ERROR {exc}")
        return

    burst_packets = count_burst_packets(bytes(data), num_mags)
    print(f"{port}: bytes={len(data)}, anyskin_burst_packets={burst_packets}")
    if data:
        preview = bytes(data[-120:]).replace(b"\r", b"\\r").replace(b"\n", b"\\n")
        print(f"{port}: tail={preview!r}")
    if burst_packets > 0:
        print(f"{port}: looks like AnySkin")
    elif len(data) > 0:
        print(f"{port}: has serial data, but it did not parse as AnySkin burst packets")
    else:
        print(f"{port}: no serial data during probe window")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ports", default="", help="Comma-separated ports. Default probes /dev/ttyACM* and /dev/ttyUSB*.")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--reset-wait-sec", type=float, default=2.0)
    parser.add_argument("--num-mags", type=int, default=5)
    args = parser.parse_args()

    ports = parse_ports(args.ports)
    if not ports:
        print("No /dev/ttyACM* or /dev/ttyUSB* ports found.")
        return 1
    for port in ports:
        read_port(port, args.baudrate, args.seconds, args.num_mags, args.reset_wait_sec)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
