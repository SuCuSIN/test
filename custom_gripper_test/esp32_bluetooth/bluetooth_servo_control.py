"""Interactive Windows Bluetooth serial terminal for the ESP32 servo driver."""

from __future__ import annotations

import argparse
import threading

import serial
from serial.tools import list_ports


def print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device}: {port.description}")


def receive(servo: serial.Serial, stopped: threading.Event) -> None:
    while not stopped.is_set():
        try:
            line = servo.readline()
        except serial.SerialException as error:
            print(f"\nConnection lost: {error}")
            stopped.set()
            return
        if line:
            print(f"\nESP32> {line.decode('utf-8', errors='replace').rstrip()}\n> ", end="")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="Bluetooth outgoing COM port, e.g. COM8")
    parser.add_argument("--list", action="store_true", help="List available COM ports")
    args = parser.parse_args()

    if args.list or not args.port:
        print_ports()
        if not args.port:
            return

    stopped = threading.Event()
    try:
        with serial.Serial(args.port, 115200, timeout=0.2) as servo:
            reader = threading.Thread(target=receive, args=(servo, stopped), daemon=True)
            reader.start()
            print("Connected. Commands:")
            print("  PING")
            print("  OPEN | CLOSE")
            print("  READ <id>")
            print("  MOVE <id> <logical_position> [speed] [acceleration]")
            print("  RAW_MOVE <id> <raw_position> [speed] [acceleration]")
            print("  SYNC_MOVE <id1> <pos1> <id2> <pos2> [speed] [acceleration]")
            print("  PAIR_MOVE <position> [speed] [acceleration]")
            print("  TORQUE_ON <id> | TORQUE_OFF <id>")
            print("  SET_ZERO")
            print("  CLEAR_OFFSET <id>")
            print("  SET_ID <current_id> <new_id>")
            print("  quit")

            while not stopped.is_set():
                command = input("> ").strip()
                if command.lower() in {"quit", "exit"}:
                    break
                if command:
                    servo.write((command + "\n").encode("ascii"))
                    servo.flush()
    except serial.SerialException as error:
        raise SystemExit(f"Could not open {args.port}: {error}") from error
    finally:
        stopped.set()


if __name__ == "__main__":
    main()
