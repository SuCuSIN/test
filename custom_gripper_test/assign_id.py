"""Safely change the ID stored in one STS3215 servo."""

import argparse
import time

from sts3215_bus import LOCK_ADDRESS, ID_ADDRESS, open_bus, ping, validate_servo_id, write_byte


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--old-id", type=int, required=True)
    parser.add_argument("--new-id", type=int, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    validate_servo_id(args.old_id)
    validate_servo_id(args.new_id)
    if args.old_id == args.new_id:
        raise SystemExit("Old ID and new ID are identical; nothing to change.")

    print("WARNING: Connect exactly ONE STS3215 servo to the bus.")
    print(f"Port: {args.port}, baudrate: {args.baudrate}")
    print(f"Requested change: ID {args.old_id} -> ID {args.new_id}")

    try:
        with open_bus(args.port, args.baudrate) as ser:
            if not ping(ser, args.old_id):
                print(f"No valid response from old ID {args.old_id}; no write performed.")
                return 1
            if ping(ser, args.new_id):
                print(f"ID {args.new_id} already responds; refusing to create an ID collision.")
                return 1

            expected = f"CHANGE {args.old_id} TO {args.new_id}"
            if input(f'Type "{expected}" to continue: ').strip() != expected:
                print("Confirmation did not match; no write performed.")
                return 1

            if not write_byte(ser, args.old_id, LOCK_ADDRESS, 0):
                print("Failed to unlock EEPROM; ID was not changed.")
                return 1
            if not write_byte(ser, args.old_id, ID_ADDRESS, args.new_id):
                print("ID write was not acknowledged. Scan both old and new IDs before retrying.")
                return 1

            time.sleep(0.2)
            if not ping(ser, args.new_id):
                print("New ID did not respond. Scan both old and new IDs before retrying.")
                return 1
            if not write_byte(ser, args.new_id, LOCK_ADDRESS, 1):
                print("New ID works, but EEPROM re-lock was not acknowledged.")
                return 2
    except OSError as exc:
        print(f"Serial port error: {exc}")
        return 2

    print(f"SUCCESS: STS3215 ID changed from {args.old_id} to {args.new_id}.")
    print("Power-cycle the servo, then scan again to confirm persistence.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
