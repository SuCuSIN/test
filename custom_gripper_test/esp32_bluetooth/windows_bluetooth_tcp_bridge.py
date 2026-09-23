"""Expose a Windows Bluetooth COM port as a tiny TCP serial bridge."""

from __future__ import annotations

import argparse
import socket
import threading

import serial


def serial_to_client(servo: serial.Serial, client: socket.socket, stopped: threading.Event, trace: bool = False) -> None:
    while not stopped.is_set():
        try:
            # Forward available bytes immediately; the TCP client frames lines.
            line = servo.read(min(4096, max(1, servo.in_waiting)))
            if line:
                if trace:
                    print(f"BT -> TCP: {line!r}", flush=True)
                client.sendall(line)
        except (OSError, serial.SerialException) as exc:
            print(f"Serial-to-client stopped: {exc}", flush=True)
            stopped.set()
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            return


def serve(port: str, host: str, tcp_port: int, trace: bool = False) -> None:
    with serial.Serial(port, 115200, timeout=0.05, write_timeout=0.2) as servo:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, tcp_port))
            server.listen(1)
            print(f"Bluetooth bridge ready: {port} -> {host}:{tcp_port}")
            while True:
                client, address = server.accept()
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"Client connected: {address}")
                stopped = threading.Event()
                reader = threading.Thread(
                    target=serial_to_client,
                    args=(servo, client, stopped, trace),
                    daemon=True,
                )
                reader.start()
                with client:
                    try:
                        while not stopped.is_set():
                            data = client.recv(4096)
                            if not data:
                                print("TCP peer closed connection", flush=True)
                                break
                            if trace:
                                print(f"TCP -> BT write requested: {data!r}", flush=True)
                            written = servo.write(data)
                            if written != len(data):
                                raise OSError(f"Short serial write: {written}/{len(data)} bytes")
                            if trace:
                                print(f"BT write returned: {written} bytes (not device ACK)", flush=True)
                            # Device ACK, not a driver-buffer flush, confirms handling.
                    except OSError as exc:
                        print(f"Client-to-serial stopped: {exc}", flush=True)
                stopped.set()
                reader.join(timeout=1.0)
                if reader.is_alive():
                    raise RuntimeError("Previous serial reader did not stop; restart bridge")
                print("Client disconnected")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=15555)
    parser.add_argument("--trace", action="store_true")
    args = parser.parse_args()
    serve(args.port, args.host, args.tcp_port, args.trace)


if __name__ == "__main__":
    main()
