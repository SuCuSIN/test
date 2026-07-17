"""Expose a Windows Bluetooth COM port as a tiny TCP serial bridge."""

from __future__ import annotations

import argparse
import socket
import threading

import serial


def serial_to_client(servo: serial.Serial, client: socket.socket, stopped: threading.Event) -> None:
    while not stopped.is_set():
        try:
            line = servo.readline()
            if line:
                client.sendall(line)
        except (OSError, serial.SerialException):
            stopped.set()
            return


def serve(port: str, host: str, tcp_port: int) -> None:
    with serial.Serial(port, 115200, timeout=0.05, write_timeout=0.2) as servo:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, tcp_port))
            server.listen(1)
            print(f"Bluetooth bridge ready: {port} -> {host}:{tcp_port}")
            while True:
                client, address = server.accept()
                print(f"Client connected: {address}")
                stopped = threading.Event()
                reader = threading.Thread(
                    target=serial_to_client,
                    args=(servo, client, stopped),
                    daemon=True,
                )
                reader.start()
                with client:
                    try:
                        while not stopped.is_set():
                            data = client.recv(4096)
                            if not data:
                                break
                            servo.write(data)
                            servo.flush()
                    except OSError:
                        pass
                stopped.set()
                print("Client disconnected")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=15555)
    args = parser.parse_args()
    serve(args.port, args.host, args.tcp_port)


if __name__ == "__main__":
    main()
