import socket
import struct
import threading
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def with_crc(body: bytes) -> bytes:
    return body + struct.pack("<H", crc16_modbus(body))


def write_multiple_request(slave_id: int, address: int, values: Iterable[int]) -> bytes:
    values = list(values)
    body = struct.pack(">BBHHB", slave_id, 0x10, address, len(values), 2 * len(values))
    body += b"".join(struct.pack(">H", value & 0xFFFF) for value in values)
    return with_crc(body)


def read_holding_request(slave_id: int, address: int, count: int) -> bytes:
    return with_crc(struct.pack(">BBHH", slave_id, 0x03, address, count))


def write_single_request(slave_id: int, address: int, value: int) -> bytes:
    return with_crc(struct.pack(">BBHH", slave_id, 0x06, address, value))


class RG6ToolTcpNode(Node):
    def __init__(self) -> None:
        super().__init__("rg6_tool_tcp")
        self.declare_parameter("robot_ip", "192.168.0.119")
        self.declare_parameter("tcp_port", 54321)
        self.declare_parameter("command_topic", "onrobot/finger_width_controller/commands")
        self.declare_parameter("slave_id", 65)
        self.declare_parameter("force", 80)
        self.declare_parameter("hold_force", 50)
        self.declare_parameter("min_width", 0.012)
        self.declare_parameter("max_width", 0.13)
        self.declare_parameter("socket_timeout", 0.35)
        self.declare_parameter("send_rate_hz", 10.0)
        self.declare_parameter("smoothing_alpha", 1.0)
        self.declare_parameter("command_deadband", 0.0015)
        self.declare_parameter("max_width_step_per_send", 0.0)
        self.declare_parameter("max_close_step_per_send", 0.012)
        self.declare_parameter("max_open_step_per_send", 0.0)
        self.declare_parameter("reversal_deadband", 0.005)
        self.declare_parameter("hold_on_grip_detected", True)
        self.declare_parameter("status_read_rate_hz", 4.0)
        self.declare_parameter("grip_hold_margin", 0.0)
        self.declare_parameter("grip_release_margin", 0.006)
        self.declare_parameter("use_stall_grip_detection", False)
        self.declare_parameter("grip_stall_time_sec", 0.30)
        self.declare_parameter("grip_stall_width_epsilon", 0.001)
        self.declare_parameter("grip_close_request_margin", 0.003)
        self.declare_parameter("actual_width_register", 267)
        self.declare_parameter("send_stop_on_grip", False)
        self.declare_parameter("send_hold_on_grip", True)
        self.declare_parameter("persistent_connection", False)

        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.tcp_port = int(self.get_parameter("tcp_port").value)
        self.slave_id = int(self.get_parameter("slave_id").value)
        self.force = float(self.get_parameter("force").value)
        self.hold_force = float(self.get_parameter("hold_force").value)
        self.min_width = float(self.get_parameter("min_width").value)
        self.max_width = float(self.get_parameter("max_width").value)
        self.socket_timeout = float(self.get_parameter("socket_timeout").value)
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.command_deadband = float(self.get_parameter("command_deadband").value)
        self.max_width_step_per_send = float(
            self.get_parameter("max_width_step_per_send").value
        )
        self.max_close_step_per_send = float(
            self.get_parameter("max_close_step_per_send").value
        )
        self.max_open_step_per_send = float(
            self.get_parameter("max_open_step_per_send").value
        )
        self.reversal_deadband = float(self.get_parameter("reversal_deadband").value)
        self.hold_on_grip_detected = bool(
            self.get_parameter("hold_on_grip_detected").value
        )
        self.status_read_rate_hz = float(self.get_parameter("status_read_rate_hz").value)
        self.grip_hold_margin = float(self.get_parameter("grip_hold_margin").value)
        self.grip_release_margin = float(self.get_parameter("grip_release_margin").value)
        self.use_stall_grip_detection = bool(
            self.get_parameter("use_stall_grip_detection").value
        )
        self.grip_stall_time_sec = float(
            self.get_parameter("grip_stall_time_sec").value
        )
        self.grip_stall_width_epsilon = float(
            self.get_parameter("grip_stall_width_epsilon").value
        )
        self.grip_close_request_margin = float(
            self.get_parameter("grip_close_request_margin").value
        )
        self.actual_width_register = int(
            self.get_parameter("actual_width_register").value
        )
        self.send_stop_on_grip = bool(self.get_parameter("send_stop_on_grip").value)
        self.send_hold_on_grip = bool(self.get_parameter("send_hold_on_grip").value)
        self.persistent_connection = bool(
            self.get_parameter("persistent_connection").value
        )
        command_topic = str(self.get_parameter("command_topic").value)

        self.raw_target_width = None
        self.target_width = None
        self.current_width = None
        self.last_sent_width = None
        self.last_direction = 0
        self.last_status_read_time = 0.0
        self.grip_detected = False
        self.grip_hold_width = None
        self.grip_stop_sent = False
        self.last_actual_width = None
        self.last_actual_width_change_time = time.monotonic()
        self.io_lock = threading.Lock()
        self.sock = None
        self.send_lock = threading.Lock()
        self.send_in_flight = False
        self.pending_send_width = None
        self.status_lock = threading.Lock()
        self.status_in_flight = False
        self.create_subscription(Float64MultiArray, command_topic, self.command_callback, 10)
        self.create_timer(1.0 / self.send_rate_hz, self.timer_callback)
        self.get_logger().info(
            f"RG6 Tool TCP node ready: {self.robot_ip}:{self.tcp_port}, topic /{command_topic}"
        )

    def destroy_node(self) -> bool:
        self.close_socket()
        return super().destroy_node()

    def command_callback(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        first_command = self.raw_target_width is None
        self.raw_target_width = max(
            self.min_width,
            min(self.max_width, float(msg.data[0])),
        )
        if self.target_width is None:
            self.target_width = self.raw_target_width
        if self.current_width is None:
            self.current_width = self.raw_target_width
        if first_command:
            self.start_send_width(self.raw_target_width)
            self.get_logger().info(
                f"Initial RG6 width command received: {self.raw_target_width:.3f} m"
            )

    def timer_callback(self) -> None:
        if self.raw_target_width is None:
            return
        if self.target_width is None:
            self.target_width = self.raw_target_width
        else:
            self.target_width = (
                (1.0 - self.smoothing_alpha) * self.target_width
                + self.smoothing_alpha * self.raw_target_width
            )

        self.start_status_read_if_needed()
        if self.grip_hold_width is not None:
            if self.raw_target_width > self.grip_hold_width + self.grip_release_margin:
                self.grip_hold_width = None
                self.grip_detected = False
                self.grip_stop_sent = False
                self.target_width = self.raw_target_width
            elif self.raw_target_width <= self.grip_hold_width + self.grip_release_margin:
                self.target_width = self.grip_hold_width
                self.current_width = self.grip_hold_width
                return

        if self.current_width is None:
            self.current_width = self.target_width

        delta = self.target_width - self.current_width
        if abs(delta) < self.command_deadband:
            return

        direction = 1 if delta > 0.0 else -1
        if (
            self.last_direction != 0
            and direction != self.last_direction
            and abs(delta) < self.reversal_deadband
        ):
            return

        if delta < 0.0 and self.max_close_step_per_send > 0.0:
            delta = max(-self.max_close_step_per_send, delta)
        elif delta > 0.0 and self.max_open_step_per_send > 0.0:
            delta = min(self.max_open_step_per_send, delta)
        elif self.max_width_step_per_send > 0.0:
            delta = max(
                -self.max_width_step_per_send,
                min(self.max_width_step_per_send, delta),
            )

        self.current_width = max(
            self.min_width,
            min(self.max_width, self.current_width + delta),
        )

        if (
            self.last_sent_width is not None
            and abs(self.current_width - self.last_sent_width) < self.command_deadband
        ):
            return

        self.start_send_width(self.current_width)
        self.last_direction = direction

    def start_status_read_if_needed(self) -> None:
        if not self.hold_on_grip_detected or self.status_read_rate_hz <= 0.0:
            return
        if self.raw_target_width is None or self.current_width is None:
            return
        if self.raw_target_width >= self.current_width:
            return

        now = time.monotonic()
        if now - self.last_status_read_time < 1.0 / self.status_read_rate_hz:
            return
        self.last_status_read_time = now

        with self.status_lock:
            if self.status_in_flight:
                return
            self.status_in_flight = True

        thread = threading.Thread(target=self.status_read_worker, daemon=True)
        thread.start()

    def status_read_worker(self) -> None:
        try:
            self.update_grip_hold()
        finally:
            with self.status_lock:
                self.status_in_flight = False

    def update_grip_hold(self) -> None:
        width_raw = self.read_register(self.actual_width_register)
        if width_raw is None:
            return

        now = time.monotonic()
        width_m = max(self.min_width, min(self.max_width, width_raw / 10000.0))
        if (
            self.last_actual_width is None
            or abs(width_m - self.last_actual_width) > self.grip_stall_width_epsilon
        ):
            self.last_actual_width = width_m
            self.last_actual_width_change_time = now

        status = self.read_register(268)
        if status is None:
            status = 0

        grip_detected = bool(status & 0x0002)
        closing_requested = (
            self.raw_target_width is not None
            and self.raw_target_width < width_m - self.grip_close_request_margin
        )
        stalled_while_closing = (
            self.use_stall_grip_detection
            and closing_requested
            and now - self.last_actual_width_change_time >= self.grip_stall_time_sec
        )

        if not grip_detected and not stalled_while_closing:
            self.grip_detected = False
            return

        reason = "grip detected" if grip_detected else "closing stalled"
        self.hold_grip(width_m, reason)

    def hold_grip(self, width_m: float, reason: str) -> None:
        if self.grip_hold_width is None or width_m > self.grip_hold_width:
            self.grip_hold_width = width_m
        self.current_width = self.grip_hold_width
        self.target_width = self.grip_hold_width
        self.last_sent_width = self.grip_hold_width
        self.grip_detected = True
        if not self.grip_stop_sent:
            if self.send_hold_on_grip:
                hold_width = max(
                    self.min_width,
                    min(self.max_width, self.grip_hold_width),
                )
                self.send_width(hold_width, force=self.hold_force)
            if self.send_stop_on_grip:
                self.send_stop(reason)
            else:
                self.get_logger().info(
                    f"RG6 {reason}; blocking further close commands until the lever opens."
                )
            self.grip_stop_sent = True

    def start_send_width(self, width_m: float) -> None:
        with self.send_lock:
            self.pending_send_width = width_m
            if self.send_in_flight:
                return
            self.send_in_flight = True

        thread = threading.Thread(target=self.send_width_worker, daemon=True)
        thread.start()

    def send_width_worker(self) -> None:
        try:
            while True:
                with self.send_lock:
                    width_m = self.pending_send_width
                    self.pending_send_width = None

                if width_m is None:
                    return

                if self.send_width(width_m):
                    self.last_sent_width = width_m
        finally:
            with self.send_lock:
                self.send_in_flight = False
                restart_width = self.pending_send_width

            if restart_width is not None:
                self.start_send_width(restart_width)

    def send_width(self, width_m: float, force: int | None = None) -> bool:
        is_hold_command = force is not None
        if (
            not is_hold_command
            and
            self.grip_hold_width is not None
            and width_m <= self.grip_hold_width + self.grip_release_margin
        ):
            return False

        if force is None:
            force = self.force

        width_register = int(round(width_m * 10000.0))
        force_register = self.force_to_register(force)
        packet = write_multiple_request(
            self.slave_id,
            0,
            [force_register, width_register, 16],
        )

        try:
            response = self.transact(packet, 8)
        except OSError as exc:
            self.get_logger().warn(f"Failed to send RG6 command: {exc}")
            return False

        if response is None:
            self.get_logger().warn("Failed to send RG6 command: no response")
            return False

        if len(response) < 8:
            self.get_logger().warn(f"Short RG6 response: {response.hex(' ')}")
            return False

        payload = response[:-2]
        received_crc = struct.unpack("<H", response[-2:])[0]
        expected_crc = crc16_modbus(payload)
        if received_crc != expected_crc:
            self.get_logger().warn(
                f"Bad RG6 CRC: expected 0x{expected_crc:04x}, got 0x{received_crc:04x}"
            )
            return False

        self.get_logger().debug(f"RG6 width command sent: {width_m:.3f} m")
        return True

    def force_to_register(self, force_n: float) -> int:
        force_n = max(0.0, min(120.0, float(force_n)))
        return int(round(force_n * 10.0))

    def send_stop(self, reason: str = "grip detected") -> None:
        packet = write_single_request(self.slave_id, 2, 8)
        try:
            self.transact(packet, 8)
        except OSError:
            return

        self.get_logger().info(
            f"RG6 {reason}; stopping close motion until the lever opens."
        )

    def read_register(self, address: int) -> int | None:
        packet = read_holding_request(self.slave_id, address, 1)
        try:
            response = self.transact(packet, 7)
        except OSError:
            return None

        if response is None or len(response) < 7:
            return None

        payload = response[:-2]
        received_crc = struct.unpack("<H", response[-2:])[0]
        if received_crc != crc16_modbus(payload):
            return None

        if response[0] != self.slave_id or response[1] != 0x03 or response[2] < 2:
            return None

        return struct.unpack(">H", response[3:5])[0]

    def transact(self, packet: bytes, response_len: int) -> bytes | None:
        with self.io_lock:
            if not self.persistent_connection:
                with socket.create_connection(
                    (self.robot_ip, self.tcp_port),
                    timeout=self.socket_timeout,
                ) as sock:
                    sock.settimeout(self.socket_timeout)
                    sock.sendall(packet)
                    return self.recv_exact(sock, response_len)

            sock = self.ensure_socket()
            try:
                sock.sendall(packet)
                return self.recv_exact(sock, response_len)
            except OSError:
                self.close_socket_unlocked()
                sock = self.ensure_socket()
                sock.sendall(packet)
                return self.recv_exact(sock, response_len)

    def ensure_socket(self) -> socket.socket:
        if self.sock is None:
            self.sock = socket.create_connection(
                (self.robot_ip, self.tcp_port),
                timeout=self.socket_timeout,
            )
            self.sock.settimeout(self.socket_timeout)
        return self.sock

    def close_socket(self) -> None:
        with self.io_lock:
            self.close_socket_unlocked()

    def close_socket_unlocked(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def recv_exact(self, sock: socket.socket, response_len: int) -> bytes:
        chunks = []
        remaining = response_len
        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                raise OSError("empty response")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)


def main() -> None:
    rclpy.init()
    node = RG6ToolTcpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
