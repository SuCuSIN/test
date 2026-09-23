"""Map GELLO gripper raw feedback to the Bluetooth STS3215 gripper."""

from __future__ import annotations

import time
from typing import List, Optional
import socket
import re

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
        self.declare_parameter("tactile_topic", "")
        self.declare_parameter("tactile_contact_threshold", 25.0)
        self.declare_parameter("tactile_baseline_samples", 50)
        self.declare_parameter("tactile_release_delta", 40.0)
        self.declare_parameter("position_report_hz", 0.0)

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
        self.tactile_contact_threshold = max(
            0.0, float(self.get_parameter("tactile_contact_threshold").value)
        )
        self.tactile_baseline_samples = max(
            1, int(self.get_parameter("tactile_baseline_samples").value)
        )
        self.tactile_release_delta = max(
            0.0, float(self.get_parameter("tactile_release_delta").value)
        )

        self.serial_port: Optional[serial.Serial] = None
        self.socket: Optional[socket.socket] = None
        self.server_socket: Optional[socket.socket] = None
        self.latest_target: Optional[float] = None
        self.filtered_target: Optional[float] = None
        self.last_command: Optional[int] = None
        self.last_connect_attempt = 0.0
        self.response_buffer = ''
        self.position_report_hz = max(0.0, float(self.get_parameter('position_report_hz').value))
        self.next_position_read = 0.0
        self.position_read_id = 1
        self.position_read_pending = None
        self.actual_publisher = self.create_publisher(Float64MultiArray, '/gello/gripper_actual_raw', 10)
        self.command_report_publisher = self.create_publisher(Float64MultiArray, '/gello/no_anyskin_command_width', 10)
        self.tactile_baseline: Optional[List[float]] = None
        self.tactile_baseline_count = 0
        self.tactile_contact_strength = 0.0
        self.tactile_latched_command: Optional[float] = None

        topic = str(self.get_parameter("topic").value)
        self.create_subscription(Float32, topic, self.raw_callback, 10)
        command_topic = str(self.get_parameter("command_topic").value)
        if command_topic:
            self.create_subscription(Float64MultiArray, command_topic, self.command_callback, 10)
        tactile_topic = str(self.get_parameter("tactile_topic").value)
        if tactile_topic:
            self.create_subscription(Float64MultiArray, tactile_topic, self.tactile_callback, 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(f"Mapping {topic} to Bluetooth gripper on {self.port}")
        if command_topic:
            self.get_logger().info(f"Also mapping width commands from {command_topic}")
        if tactile_topic:
            self.get_logger().info(
                f"Using tactile contact safety from {tactile_topic} "
                f"(threshold={self.tactile_contact_threshold:.2f})"
            )

    def raw_callback(self, message: Float32) -> None:
        raw = float(message.data)
        if self.latest_target is None:
            self.get_logger().info(f"First GELLO lever input received: raw={raw}")
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

    def tactile_callback(self, message: Float64MultiArray) -> None:
        if not message.data:
            return
        values = [float(value) for value in message.data]
        if self.tactile_baseline is None:
            self.tactile_baseline = values
            self.tactile_baseline_count = 1
            return

        if len(values) != len(self.tactile_baseline):
            self.tactile_baseline = values
            self.tactile_baseline_count = 1
            self.tactile_latched_command = None
            self.get_logger().warn("Tactile vector size changed; recalibrating baseline.")
            return

        if self.tactile_baseline_count < self.tactile_baseline_samples:
            count = self.tactile_baseline_count
            self.tactile_baseline = [
                (baseline * count + value) / (count + 1)
                for baseline, value in zip(self.tactile_baseline, values)
            ]
            self.tactile_baseline_count += 1
            return

        squared_delta = sum(
            (value - baseline) ** 2
            for baseline, value in zip(self.tactile_baseline, values)
        )
        self.tactile_contact_strength = squared_delta ** 0.5

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
                data = self.socket.recv(4096)
                if not data:
                    raise ConnectionError("Bluetooth TCP peer closed the connection")
                return data.decode(errors="replace")
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
        self.response_buffer = ''
        self.position_read_pending = None

    def record_response(self, response):
        match = re.fullmatch(r'POSITION id=([12]) logical=-?\d+ raw=(\d+) status=0x0+', response)
        if match and 0 <= int(match[2]) <= 4095:
            message = Float64MultiArray()
            message.data = [float(match[1]), float(match[2])]
            self.actual_publisher.publish(message)
            if self.position_read_pending == int(match[1]):
                self.position_read_pending = None

    def poll_position_report(self):
        now = time.monotonic()
        if self.position_report_hz <= 0 or now < self.next_position_read:
            return
        if self.position_read_pending is not None:
            self.get_logger().warning('Position report timed out; no measurement recorded', throttle_duration_sec=5.0)
            self.position_read_pending = None
            self.next_position_read = now + 1.0
            return
        self.write_command(f'READ {self.position_read_id}\n')
        self.position_read_pending = self.position_read_id
        self.position_read_id = 3 - self.position_read_id
        self.next_position_read = now + max(0.2, 1.0 / self.position_report_hz)

    def update(self) -> None:
        if not self.is_connected():
            if not self.connect():
                return
        try:
            responses = self.read_available()
            self.response_buffer += responses
            while '\n' in self.response_buffer:
                response, self.response_buffer = self.response_buffer.split('\n', 1)
                response = response.strip()
                if response:
                    self.record_response(response)
                    self.get_logger().info(response)
            if len(self.response_buffer) > 4096:
                self.response_buffer = ''
            self.poll_position_report()

            if self.latest_target is None:
                return
            target = self.apply_tactile_safety(self.latest_target)
            if self.filtered_target is None:
                self.filtered_target = target
            else:
                desired = self.filtered_target + self.alpha * (
                    target - self.filtered_target
                )
                delta = max(-self.max_step, min(self.max_step, desired - self.filtered_target))
                self.filtered_target += delta

            command = int(round(max(self.output_min, min(self.output_max, self.filtered_target))))
            if self.last_command is not None and abs(command - self.last_command) < self.deadband:
                return
            payload = f"PAIR_MOVE {command} {self.speed} {self.acceleration}\n"
            self.write_command(payload)
            width = self.input_max_width - (command / self.output_max) * (self.input_max_width - self.input_min_width)
            report = Float64MultiArray()
            report.data = [float(width)]
            self.command_report_publisher.publish(report)
            self.get_logger().info(
                f"Gripper TX (not motor ACK): {payload.strip()}", throttle_duration_sec=1.0)
            self.last_command = command
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(f"Bluetooth connection lost: {error}")
            self.close_connection()
            self.last_command = None
            self.latest_target = None
            self.filtered_target = None

    def apply_tactile_safety(self, target: float) -> float:
        if self.tactile_baseline_count < self.tactile_baseline_samples:
            return target

        if self.tactile_latched_command is not None:
            if target <= self.tactile_latched_command - self.tactile_release_delta:
                self.get_logger().info("Tactile grip hold released by open command.")
                self.tactile_latched_command = None
                return target
            return min(target, self.tactile_latched_command)

        reference = (
            self.filtered_target
            if self.filtered_target is not None
            else float(self.last_command or self.output_min)
        )
        is_closing = target > reference
        if is_closing and self.tactile_contact_strength >= self.tactile_contact_threshold:
            self.tactile_latched_command = reference
            self.get_logger().info(
                "Tactile contact detected; holding gripper command at "
                f"{self.tactile_latched_command:.0f} "
                f"(strength={self.tactile_contact_strength:.2f})."
            )
            return self.tactile_latched_command

        return target

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
