"""Connect a Windows Bluetooth COM port to a TCP server running in WSL."""

from __future__ import annotations

import argparse
import socket
import threading
import time

import serial


def serial_to_socket(servo: serial.Serial, sock: socket.socket, stopped: threading.Event) -> None:
    while not stopped.is_set():
        try:
            line = servo.readline()
            if line:
                print(f"ESP32> {line.decode('utf-8', errors='replace').rstrip()}")
                sock.sendall(line)
        except TypeError:
            # pyserial on Windows can raise this while another thread is closing the port.
            stopped.set()
            return
        except (OSError, serial.SerialException):
            stopped.set()
            return


def run(port: str, host: str, tcp_port: int) -> None:
    while True:
        try:
            with serial.Serial(port, 115200, timeout=0.05, write_timeout=0.2) as servo:
                print(f"Opened {port}. Connecting to {host}:{tcp_port}...")
                with socket.create_connection((host, tcp_port), timeout=5.0) as sock:
                    sock.settimeout(None)
                    print(f"Connected {port} -> {host}:{tcp_port}")
                    stopped = threading.Event()
                    reader = threading.Thread(
                        target=serial_to_socket,
                        args=(servo, sock, stopped),
                        daemon=True,
                    )
                    reader.start()
                    while not stopped.is_set():
                        try:
                            data = sock.recv(4096)
                            if not data:
                                break
                            for command in data.decode("ascii", errors="replace").splitlines():
                                if command:
                                    print(f"ROS> {command}")
                            servo.write(data)
                            servo.flush()
                        except socket.timeout:
                            continue
        except (OSError, serial.SerialException) as error:
            print(f"Bridge disconnected: {error}")
        print("Retrying in 2 seconds...")
        time.sleep(2.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--host", required=True)
    parser.add_argument("--tcp-port", type=int, default=15555)
    args = parser.parse_args()
    run(args.port, args.host, args.tcp_port)


if __name__ == "__main__":
    main()
