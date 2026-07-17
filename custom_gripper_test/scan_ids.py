"""Scan a serial bus for responding STS3215 IDs."""

import argparse

from sts3215_bus import MAX_SERVO_ID, MIN_SERVO_ID, open_bus, ping


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--end", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not MIN_SERVO_ID <= args.start <= args.end <= MAX_SERVO_ID:
        raise SystemExit(f"ID range must satisfy 0 <= start <= end <= {MAX_SERVO_ID}")

    found = []
    print(f"Scanning {args.port} at {args.baudrate} baud (IDs {args.start}..{args.end})")
    try:
        with open_bus(args.port, args.baudrate) as ser:
            for servo_id in range(args.start, args.end + 1):
                if ping(ser, servo_id):
                    found.append(servo_id)
                    print(f"Found servo ID {servo_id}")
    except OSError as exc:
        print(f"Serial port error: {exc}")
        return 2

    if not found:
        print("No responding servo found.")
        return 1
    print("Detected IDs:", ", ".join(map(str, found)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

