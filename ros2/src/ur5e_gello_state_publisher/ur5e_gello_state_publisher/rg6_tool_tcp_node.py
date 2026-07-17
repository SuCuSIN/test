import socket
import struct
import threading
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    import serial
except ImportError:
    serial = None


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
        self.declare_parameter("transport", "tcp")
        self.declare_parameter("robot_ip", "192.168.0.119")
        self.declare_parameter("tcp_port", 54321)
        self.declare_parameter("serial_device", "/dev/ttyUSB0")
        self.declare_parameter("serial_baudrate", 1000000)
        self.declare_parameter("serial_parity", "E")
        self.declare_parameter("serial_stopbits", 1)
        self.declare_parameter("command_topic", "onrobot/finger_width_controller/commands")
        self.declare_parameter("slave_id", 65)
        self.declare_parameter("force", 80)
        self.declare_parameter("hold_force", 50)
        self.declare_parameter("min_width", 0.012)
        self.declare_parameter("max_width", 0.13)
        self.declare_parameter("socket_timeout", 0.35)
        self.declare_parameter("inter_transaction_delay_sec", 0.03)
        self.declare_parameter("send_rate_hz", 10.0)
        self.declare_parameter("command_settle_sec", 0.0)
        self.declare_parameter("smoothing_alpha", 1.0)
        self.declare_parameter("command_deadband", 0.0015)
        self.declare_parameter("send_deadband", 0.006)
        self.declare_parameter("failed_send_cooldown_sec", 1.0)
        self.declare_parameter("max_width_step_per_send", 0.0)
        self.declare_parameter("max_close_step_per_send", 0.012)
        self.declare_parameter("max_open_step_per_send", 0.0)
        self.declare_parameter("reversal_deadband", 0.005)
        self.declare_parameter("close_latch_release_m", 0.020)
        self.declare_parameter("open_confirm_cycles", 6)
        self.declare_parameter("open_release_width", 0.10)
        self.declare_parameter("hold_on_grip_detected", True)
        self.declare_parameter("status_read_rate_hz", 4.0)
        self.declare_parameter("grip_hold_margin", 0.0)
        self.declare_parameter("grip_release_margin", 0.006)
        self.declare_parameter("use_stall_grip_detection", False)
        self.declare_parameter("grip_stall_time_sec", 0.30)
        self.declare_parameter("grip_stall_width_epsilon", 0.001)
        self.declare_parameter("grip_close_request_margin", 0.003)
        self.declare_parameter("actual_width_register", 267)
        self.declare_parameter("status_register", 260)
        self.declare_parameter("respect_busy_status", False)
        self.declare_parameter("send_stop_on_grip", False)
        self.declare_parameter("send_hold_on_grip", False)
        self.declare_parameter("persistent_connection", False)
        self.declare_parameter("require_response", True)
        self.declare_parameter("reconnect_after_failures", 3)
        self.declare_parameter("reconnect_cooldown_sec", 0.8)

        self.transport = str(self.get_parameter("transport").value).lower()
        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.tcp_port = int(self.get_parameter("tcp_port").value)
        self.serial_device = str(self.get_parameter("serial_device").value)
        self.serial_baudrate = int(self.get_parameter("serial_baudrate").value)
        self.serial_parity = str(self.get_parameter("serial_parity").value).upper()
        self.serial_stopbits = int(self.get_parameter("serial_stopbits").value)
        self.slave_id = int(self.get_parameter("slave_id").value)
        self.force = float(self.get_parameter("force").value)
        self.hold_force = float(self.get_parameter("hold_force").value)
        self.min_width = float(self.get_parameter("min_width").value)
        self.max_width = float(self.get_parameter("max_width").value)
        self.socket_timeout = float(self.get_parameter("socket_timeout").value)
        self.inter_transaction_delay_sec = max(
            0.0,
            float(self.get_parameter("inter_transaction_delay_sec").value),
        )
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.command_settle_sec = max(
            0.0,
            float(self.get_parameter("command_settle_sec").value),
        )
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.command_deadband = float(self.get_parameter("command_deadband").value)
        self.send_deadband = max(0.0, float(self.get_parameter("send_deadband").value))
        self.failed_send_cooldown_sec = max(
            0.0,
            float(self.get_parameter("failed_send_cooldown_sec").value),
        )
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
        self.close_latch_release_m = float(
            self.get_parameter("close_latch_release_m").value
        )
        self.open_confirm_cycles = max(
            1,
            int(self.get_parameter("open_confirm_cycles").value),
        )
        self.open_release_width = max(
            self.min_width,
            min(self.max_width, float(self.get_parameter("open_release_width").value)),
        )
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
        self.status_register = int(self.get_parameter("status_register").value)
        self.respect_busy_status = bool(
            self.get_parameter("respect_busy_status").value
        )
        self.send_stop_on_grip = bool(self.get_parameter("send_stop_on_grip").value)
        self.send_hold_on_grip = bool(self.get_parameter("send_hold_on_grip").value)
        self.persistent_connection = bool(
            self.get_parameter("persistent_connection").value
        )
        self.require_response = bool(self.get_parameter("require_response").value)
        self.reconnect_after_failures = max(
            1,
            int(self.get_parameter("reconnect_after_failures").value),
        )
        self.reconnect_cooldown_sec = max(
            0.0,
            float(self.get_parameter("reconnect_cooldown_sec").value),
        )
        command_topic = str(self.get_parameter("command_topic").value)

        self.raw_target_width = None
        self.last_raw_target_change_time = time.monotonic()
        self.target_width = None
        self.current_width = None
        self.last_sent_width = None
        self.last_direction = 0
        self.close_latch_width = None
        self.open_confirm_count = 0
        self.last_status_read_time = 0.0
        self.grip_detected = False
        self.grip_hold_width = None
        self.grip_release_confirm_count = 0
        self.grip_stop_sent = False
        self.last_actual_width = None
        self.last_actual_width_change_time = time.monotonic()
        self.io_lock = threading.Lock()
        self.sock = None
        self.serial_port = None
        self.send_lock = threading.Lock()
        self.send_in_flight = False
        self.pending_send_width = None
        self.send_blocked_until = 0.0
        self.consecutive_send_failures = 0
        self.last_reconnect_log_time = 0.0
        self.status_lock = threading.Lock()
        self.status_in_flight = False
        self.last_transaction_time = 0.0
        self.create_subscription(Float64MultiArray, command_topic, self.command_callback, 10)
        self.create_timer(1.0 / self.send_rate_hz, self.timer_callback)
        if self.uses_serial_transport():
            self.get_logger().info(
                f"RG6 serial RTU node ready: {self.serial_device} @ {self.serial_baudrate}, "
                f"topic /{command_topic}"
            )
        else:
            self.get_logger().info(
                f"RG6 Tool TCP node ready: {self.robot_ip}:{self.tcp_port}, "
                f"topic /{command_topic}"
            )

    def destroy_node(self) -> bool:
        self.close_connection()
        return super().destroy_node()

    def command_callback(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        first_command = self.raw_target_width is None
        previous_raw_target = self.raw_target_width
        requested_width = max(
            self.min_width,
            min(self.max_width, float(msg.data[0])),
        )

        if previous_raw_target is not None:
            if requested_width < previous_raw_target - self.command_deadband:
                self.open_confirm_count = 0
                if self.close_latch_width is None:
                    self.close_latch_width = requested_width
                else:
                    self.close_latch_width = min(
                        self.close_latch_width,
                        requested_width,
                    )
            elif (
                self.close_latch_width is not None
                and requested_width < self.close_latch_width + self.close_latch_release_m
            ):
                self.open_confirm_count = 0
                requested_width = self.close_latch_width
            elif (
                self.close_latch_width is not None
                and requested_width >= self.close_latch_width + self.close_latch_release_m
            ):
                release_width = max(
                    self.open_release_width,
                    self.close_latch_width + self.close_latch_release_m,
                )
                if requested_width >= release_width:
                    self.close_latch_width = None
                    self.open_confirm_count = 0
                else:
                    self.open_confirm_count = 0
                    requested_width = self.close_latch_width

        if (
            previous_raw_target is not None
            and abs(requested_width - previous_raw_target) < self.command_deadband
        ):
            return

        self.raw_target_width = max(
            self.min_width,
            min(self.max_width, requested_width),
        )
        self.last_raw_target_change_time = time.monotonic()
        self.get_logger().info(
            f"RG6 target command accepted: {self.raw_target_width:.3f} m"
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
        now = time.monotonic()
        if self.target_width is None:
            self.target_width = self.raw_target_width
        else:
            self.target_width = (
                (1.0 - self.smoothing_alpha) * self.target_width
                + self.smoothing_alpha * self.raw_target_width
            )

        if (
            self.command_settle_sec > 0.0
            and now - self.last_raw_target_change_time < self.command_settle_sec
        ):
            return

        self.start_status_read_if_needed()
        if self.grip_hold_width is not None:
            release_width = max(
                self.open_release_width,
                self.grip_hold_width + self.grip_release_margin,
            )
            if self.raw_target_width >= release_width:
                self.grip_release_confirm_count += 1
            else:
                self.grip_release_confirm_count = 0

            if self.grip_release_confirm_count >= self.open_confirm_cycles:
                self.grip_hold_width = None
                self.grip_detected = False
                self.grip_stop_sent = False
                self.grip_release_confirm_count = 0
                self.close_latch_width = None
                self.target_width = self.raw_target_width
            else:
                self.target_width = self.grip_hold_width
                self.current_width = self.grip_hold_width
                return

        if self.current_width is None:
            self.current_width = self.target_width

        delta = self.target_width - self.current_width
        if abs(delta) < self.command_deadband:
            self.retry_unsent_width_if_needed()
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

    def retry_unsent_width_if_needed(self) -> None:
        if self.current_width is None:
            return
        if self.last_sent_width is None:
            self.start_send_width(self.current_width)
            return
        if abs(self.current_width - self.last_sent_width) >= self.send_deadband:
            self.start_send_width(self.current_width)

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
        registers = self.read_registers(self.actual_width_register, 2)
        if registers is None:
            return
        width_raw, status = registers

        now = time.monotonic()
        width_m = max(self.min_width, min(self.max_width, width_raw / 10000.0))
        if (
            self.last_actual_width is None
            or abs(width_m - self.last_actual_width) > self.grip_stall_width_epsilon
        ):
            self.last_actual_width = width_m
            self.last_actual_width_change_time = now

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

        close_blocked = (
            closing_requested
            and self.last_actual_width is not None
            and width_m >= self.last_actual_width - self.grip_stall_width_epsilon
            and now - self.last_actual_width_change_time >= self.grip_stall_time_sec
        )

        if not grip_detected and not stalled_while_closing and not close_blocked:
            self.grip_detected = False
            return

        if grip_detected:
            reason = "grip detected"
        elif stalled_while_closing:
            reason = "closing stalled"
        else:
            reason = "close blocked"
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

            if time.monotonic() < self.send_blocked_until:
                return
            if (
                self.last_sent_width is not None
                and abs(width_m - self.last_sent_width) < self.send_deadband
            ):
                return
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
                    self.consecutive_send_failures = 0
                else:
                    self.handle_send_failure()
        finally:
            with self.send_lock:
                self.send_in_flight = False
                restart_width = self.pending_send_width

            if restart_width is not None:
                self.start_send_width(restart_width)

    def handle_send_failure(self) -> None:
        self.consecutive_send_failures += 1
        now = time.monotonic()
        cooldown = self.failed_send_cooldown_sec

        if self.consecutive_send_failures >= self.reconnect_after_failures:
            self.close_connection()
            self.consecutive_send_failures = 0
            cooldown = max(cooldown, self.reconnect_cooldown_sec)
            if now - self.last_reconnect_log_time > 2.0:
                self.get_logger().warn(
                    "RG6 communication failed repeatedly; reconnecting and retrying latest command."
                )
                self.last_reconnect_log_time = now

        self.send_blocked_until = now + cooldown

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

        if self.respect_busy_status and self.is_gripper_busy():
            return False

        width_register = int(round(width_m * 10000.0))
        force_register = self.force_to_register(force)
        packet = write_multiple_request(
            self.slave_id,
            0,
            [force_register, width_register, 16],
        )

        try:
            if not self.require_response:
                self.send_packet_without_response(packet)
                self.get_logger().debug(f"RG6 width command sent: {width_m:.3f} m")
                return True

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
        registers = self.read_registers(address, 1)
        if registers is None:
            return None
        return registers[0]

    def is_gripper_busy(self) -> bool:
        status = self.read_register(self.status_register)
        if status is None:
            return False
        return bool(status & 0x0001)

    def read_registers(self, address: int, count: int) -> list[int] | None:
        packet = read_holding_request(self.slave_id, address, count)
        try:
            response = self.transact(packet, 5 + 2 * count)
        except OSError:
            return None

        if response is None or len(response) < 5 + 2 * count:
            return None

        payload = response[:-2]
        received_crc = struct.unpack("<H", response[-2:])[0]
        if received_crc != crc16_modbus(payload):
            return None

        byte_count = 2 * count
        if response[0] != self.slave_id or response[1] != 0x03 or response[2] < byte_count:
            return None

        return [
            struct.unpack(">H", response[3 + 2 * index : 5 + 2 * index])[0]
            for index in range(count)
        ]

    def transact(self, packet: bytes, response_len: int) -> bytes | None:
        with self.io_lock:
            self.wait_for_transaction_spacing()
            if self.uses_serial_transport():
                response = self.transact_serial(packet, response_len)
                self.last_transaction_time = time.monotonic()
                return response

            if not self.persistent_connection:
                with socket.create_connection(
                    (self.robot_ip, self.tcp_port),
                    timeout=self.socket_timeout,
                ) as sock:
                    sock.settimeout(self.socket_timeout)
                    sock.sendall(packet)
                    response = self.recv_valid_frame(sock, response_len)
                    self.last_transaction_time = time.monotonic()
                    return response

            sock = self.ensure_socket()
            try:
                sock.sendall(packet)
                response = self.recv_valid_frame(sock, response_len)
                self.last_transaction_time = time.monotonic()
                return response
            except OSError:
                self.close_socket_unlocked()
                sock = self.ensure_socket()
                sock.sendall(packet)
                response = self.recv_valid_frame(sock, response_len)
                self.last_transaction_time = time.monotonic()
                return response

    def send_packet_without_response(self, packet: bytes) -> None:
        with self.io_lock:
            self.wait_for_transaction_spacing()
            if self.uses_serial_transport():
                port = self.ensure_serial()
                try:
                    port.reset_input_buffer()
                    port.write(packet)
                    port.flush()
                except OSError:
                    self.close_serial_unlocked()
                    port = self.ensure_serial()
                    port.reset_input_buffer()
                    port.write(packet)
                    port.flush()
                self.last_transaction_time = time.monotonic()
                return

            if not self.persistent_connection:
                with socket.create_connection(
                    (self.robot_ip, self.tcp_port),
                    timeout=self.socket_timeout,
                ) as sock:
                    sock.settimeout(self.socket_timeout)
                    sock.sendall(packet)
                    time.sleep(0.01)
                    self.last_transaction_time = time.monotonic()
                    return

            sock = self.ensure_socket()
            try:
                sock.sendall(packet)
                self.last_transaction_time = time.monotonic()
            except OSError:
                self.close_socket_unlocked()
                sock = self.ensure_socket()
                sock.sendall(packet)
                self.last_transaction_time = time.monotonic()

    def wait_for_transaction_spacing(self) -> None:
        elapsed = time.monotonic() - self.last_transaction_time
        remaining = self.inter_transaction_delay_sec - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

    def uses_serial_transport(self) -> bool:
        return self.transport in ("rs485", "serial", "tool_io")

    def transact_serial(self, packet: bytes, response_len: int) -> bytes:
        port = self.ensure_serial()
        try:
            port.reset_input_buffer()
            port.write(packet)
            port.flush()
            return self.recv_serial_valid_frame(port, response_len)
        except OSError:
            self.close_serial_unlocked()
            port = self.ensure_serial()
            port.reset_input_buffer()
            port.write(packet)
            port.flush()
            return self.recv_serial_valid_frame(port, response_len)

    def ensure_serial(self):
        if serial is None:
            raise OSError(
                "pyserial is required for transport=rs485. Install python3-serial."
            )
        if self.serial_port is None:
            parity = {
                "N": serial.PARITY_NONE,
                "E": serial.PARITY_EVEN,
                "O": serial.PARITY_ODD,
            }.get(self.serial_parity, serial.PARITY_EVEN)
            stopbits = (
                serial.STOPBITS_TWO
                if self.serial_stopbits == 2
                else serial.STOPBITS_ONE
            )
            self.serial_port = serial.Serial(
                self.serial_device,
                baudrate=self.serial_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=parity,
                stopbits=stopbits,
                timeout=self.socket_timeout,
                write_timeout=self.socket_timeout,
            )
        return self.serial_port

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

    def close_connection(self) -> None:
        with self.io_lock:
            self.close_socket_unlocked()
            self.close_serial_unlocked()

    def close_socket_unlocked(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def close_serial_unlocked(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except OSError:
                pass
            self.serial_port = None

    def recv_valid_frame(self, sock: socket.socket, response_len: int) -> bytes:
        buffer = bytearray()
        deadline = time.monotonic() + self.socket_timeout
        saw_data = False

        while time.monotonic() < deadline:
            remaining_time = max(0.01, deadline - time.monotonic())
            sock.settimeout(remaining_time)
            try:
                chunk = sock.recv(max(1, response_len - len(buffer)))
            except socket.timeout:
                break

            if not chunk:
                if saw_data:
                    break
                raise OSError("empty response")

            saw_data = True
            buffer.extend(chunk)

            while len(buffer) >= response_len:
                frame = bytes(buffer[:response_len])
                payload = frame[:-2]
                received_crc = struct.unpack("<H", frame[-2:])[0]
                if received_crc == crc16_modbus(payload):
                    return frame
                del buffer[0]

        if not saw_data:
            raise OSError("timed out")
        raise OSError("bad or incomplete response")

    def recv_serial_valid_frame(self, port, response_len: int) -> bytes:
        buffer = bytearray()
        deadline = time.monotonic() + self.socket_timeout
        saw_data = False

        while time.monotonic() < deadline:
            chunk = port.read(max(1, response_len - len(buffer)))
            if not chunk:
                break

            saw_data = True
            buffer.extend(chunk)

            while len(buffer) >= response_len:
                frame = bytes(buffer[:response_len])
                payload = frame[:-2]
                received_crc = struct.unpack("<H", frame[-2:])[0]
                if received_crc == crc16_modbus(payload):
                    return frame
                del buffer[0]

        if not saw_data:
            raise OSError("timed out")
        raise OSError("bad or incomplete response")


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
