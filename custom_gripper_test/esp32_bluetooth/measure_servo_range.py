"""Measure STS3215 raw position ranges while moving torque-released servos."""

from __future__ import annotations

import argparse
import re
import time

import serial


POSITION_PATTERN = re.compile(r"POSITION id=(\d+) logical=(-?\d+) raw=(-?\d+)")


def request(servo: serial.Serial, command: str) -> str:
    servo.reset_input_buffer()
    servo.write((command + "\n").encode("ascii"))
    servo.flush()
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        line = servo.readline().decode("utf-8", errors="replace").strip()
        if line:
            return line
    raise TimeoutError(f"No response to: {command}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Bluetooth outgoing COM port")
    parser.add_argument("--ids", nargs="+", type=int, default=[1, 2])
    parser.add_argument("--interval", type=float, default=0.1)
    args = parser.parse_args()

    ranges = {servo_id: [4096, -1] for servo_id in args.ids}
    with serial.Serial(args.port, 115200, timeout=0.1) as servo:
        time.sleep(0.5)
        for servo_id in args.ids:
            response = request(servo, f"TORQUE_OFF {servo_id}")
            if "complete" not in response:
                raise SystemExit(response)

        print("Torque is OFF. Move the servos by hand; press Ctrl+C when finished.")
        try:
            while True:
                readings = []
                for servo_id in args.ids:
                    response = request(servo, f"READ {servo_id}")
                    match = POSITION_PATTERN.search(response)
                    if match is None:
                        raise RuntimeError(response)
                    raw = int(match.group(3))
                    limits = ranges[servo_id]
                    limits[0] = min(limits[0], raw)
                    limits[1] = max(limits[1], raw)
                    readings.append(
                        f"ID {servo_id}: raw={raw:4d} range={limits[0]:4d}..{limits[1]:4d}"
                    )
                print("\r" + " | ".join(readings), end="", flush=True)
                time.sleep(max(0.02, args.interval))
        except KeyboardInterrupt:
            print("\n\nMeasured raw ranges:")
            for servo_id, (minimum, maximum) in ranges.items():
                print(f"ID {servo_id}: min={minimum}, max={maximum}")
            print("Torque remains OFF. Use TORQUE_ON only after setting safe limits.")


if __name__ == "__main__":
    main()
