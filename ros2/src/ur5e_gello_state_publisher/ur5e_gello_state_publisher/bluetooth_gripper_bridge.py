"""Map GELLO gripper raw feedback to the Bluetooth STS3215 gripper."""

from __future__ import annotations

import time
from typing import Optional
import socket

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32, Float64MultiArray


class BluetoothGripperBridge(Node):
    def __init__(self) -> None:
        super().__init__("bluetooth_gripper_bridge")
        self.declare_parameter("port", "COM10")
        self.declare_parameter("topic", "gello/gripper_raw")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("controller_min_raw", 3459.0)
        self.declare_parameter("controller_max_raw", 4000.0)
        self.declare_parameter("controller_wrap_raw", 700.0)
        self.declare_parameter("encoder_ticks", 4096.0)
        self.declare_parameter("input_min_width", 0.012)
        self.declare_parameter("input_max_width", 0.13)
        self.declare_parameter("output_min", 0)
        self.declare_parameter("output_max", 500)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("smoothing_alpha", 0.25)
        self.declare_parameter("max_step_per_update", 20.0)
        self.declare_parameter("command_deadband", 2)
        self.declare_parameter("speed", 200)
        self.declare_parameter("acceleration", 20)

        self.port = str(self.get_parameter("port").value)
        self.minimum_raw = float(self.get_parameter("controller_min_raw").value)
        self.maximum_raw = float(self.get_parameter("controller_max_raw").value)
        self.wrap_raw = float(self.get_parameter("controller_wrap_raw").value)
        self.encoder_ticks = float(self.get_parameter("encoder_ticks").value)
        self.output_min = int(self.get_parameter("output_min").value)
        self.output_max = int(self.get_parameter("output_max").value)
        self.input_min_width = float(self.get_parameter("input_min_width").value)
        self.input_max_width = float(self.get_parameter("input_max_width").value)
        self.alpha = max(0.0, min(1.0, float(self.get_parameter("smoothing_alpha").value)))
        self.max_step = max(1.0, float(self.get_parameter("max_step_per_update").value))
        self.deadband = max(0, int(self.get_parameter("command_deadband").value))
        self.speed = int(self.get_parameter("speed").value)
        self.acceleration = int(self.get_parameter("acceleration").value)

        self.serial_port: Optional[serial.Serial] = None
        self.socket: Optional[socket.socket] = None
        self.server_socket: Optional[socket.socket] = None
        self.latest_target: Optional[float] = None
        self.filtered_target: Optional[float] = None
        self.last_command: Optional[int] = None
        self.last_connect_attempt = 0.0

        topic = str(self.get_parameter("topic").value)
        self.create_subscription(Float32, topic, self.raw_callback, 10)
        command_topic = str(self.get_parameter("command_topic").value)
        if command_topic:
            self.create_subscription(Float64MultiArray, command_topic, self.command_callback, 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(f"Mapping {topic} to Bluetooth gripper on {self.port}")
        if command_topic:
            self.get_logger().info(f"Also mapping width commands from {command_topic}")

    def raw_callback(self, message: Float32) -> None:
        raw = float(message.data)
        maximum = self.maximum_raw
        if self.wrap_raw >= 0.0:
            if raw <= self.wrap_raw:
                raw += self.encoder_ticks
            if maximum <= self.minimum_raw:
                maximum += self.encoder_ticks
        span = maximum - self.minimum_raw
        if span == 0.0:
            return
        normalized = max(0.0, min(1.0, (raw - self.minimum_raw) / span))
        # Higher GELLO raw means open, while PAIR_MOVE 0 is open.
        self.latest_target = self.output_max * (1.0 - normalized)

    def command_callback(self, message: Float64MultiArray) -> None:
        if not message.data:
            return
        width = float(message.data[0])
        span = self.input_max_width - self.input_min_width
        if span == 0.0:
            return
        normalized = max(0.0, min(1.0, (width - self.input_min_width) / span))
        # Width max is open, while PAIR_MOVE 0 is open.
        self.latest_target = self.output_max * (1.0 - normalized)

    def connect(self) -> bool:
        now = time.monotonic()
        if now - self.last_connect_attempt < 2.0:
            return False
        self.last_connect_attempt = now
        try:
            if self.port.startswith("listen://"):
                host_port = self.port[len("listen://") :]
                host, port = host_port.rsplit(":", 1)
                if self.server_socket is None:
                    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    self.server_socket.bind((host, int(port)))
                    self.server_socket.listen(1)
                    self.server_socket.settimeout(0.2)
                    self.get_logger().info(f"Waiting for Bluetooth TCP client on {host}:{port}")
                self.socket, address = self.server_socket.accept()
                self.socket.settimeout(0.0)
                self.get_logger().info(f"Bluetooth TCP client connected from {address}")
            elif self.port.startswith("tcp://"):
                host_port = self.port[len("tcp://") :]
                host, port = host_port.rsplit(":", 1)
                self.socket = socket.create_connection((host, int(port)), timeout=1.0)
                self.socket.settimeout(0.0)
            else:
                self.serial_port = serial.Serial(
                    self.port, 115200, timeout=0, write_timeout=0.2
                )
            self.get_logger().info(f"Bluetooth connected on {self.port}")
            return True
        except (OSError, serial.SerialException) as error:
            self.get_logger().warning(f"Bluetooth connect failed: {error}")
            self.serial_port = None
            self.socket = None
            return False

    def is_connected(self) -> bool:
        return (
            self.socket is not None
            or (self.serial_port is not None and self.serial_port.is_open)
        )

    def read_available(self) -> str:
        if self.socket is not None:
            try:
                return self.socket.recv(4096).decode(errors="replace")
            except BlockingIOError:
                return ""
        if self.serial_port is None:
            return ""
        waiting = self.serial_port.in_waiting
        if not waiting:
            return ""
        return self.serial_port.read(waiting).decode(errors="replace")

    def write_command(self, payload: str) -> None:
        data = payload.encode("ascii")
        if self.socket is not None:
            self.socket.sendall(data)
            return
        if self.serial_port is None:
            raise serial.SerialException("serial port is not connected")
        self.serial_port.write(data)

    def close_connection(self) -> None:
        if self.serial_port is not None:
            self.serial_port.close()
        if self.socket is not None:
            self.socket.close()
        self.serial_port = None
        self.socket = None

    def update(self) -> None:
        if not self.is_connected():
            if not self.connect():
                return
        try:
            responses = self.read_available()
            for response in responses.splitlines():
                if response:
                    self.get_logger().info(response)

            if self.latest_target is None:
                return
            if self.filtered_target is None:
                self.filtered_target = self.latest_target
            else:
                desired = self.filtered_target + self.alpha * (
                    self.latest_target - self.filtered_target
                )
                delta = max(-self.max_step, min(self.max_step, desired - self.filtered_target))
                self.filtered_target += delta

            command = int(round(max(self.output_min, min(self.output_max, self.filtered_target))))
            if self.last_command is not None and abs(command - self.last_command) < self.deadband:
                return
            payload = f"PAIR_MOVE {command} {self.speed} {self.acceleration}\n"
            self.write_command(payload)
            self.last_command = command
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(f"Bluetooth connection lost: {error}")
            self.close_connection()
            self.last_command = None

    def destroy_node(self) -> bool:
        self.close_connection()
        if self.server_socket is not None:
            self.server_socket.close()
            self.server_socket = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BluetoothGripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
