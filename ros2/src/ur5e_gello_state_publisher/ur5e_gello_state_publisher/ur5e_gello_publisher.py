import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _add_local_scservo_sdk_path() -> None:
    roots = [Path.cwd(), Path(__file__).resolve()]
    for root in list(roots):
        roots.extend(root.parents)

    for root in roots:
        site_packages = root / ".venv" / "Lib" / "site-packages"
        if (site_packages / "scservo_sdk").exists():
            site_packages_text = str(site_packages)
            if site_packages_text not in sys.path:
                sys.path.insert(0, site_packages_text)
            return


_add_local_scservo_sdk_path()

try:
    from scservo_sdk import GroupSyncRead, PacketHandler, PortHandler
except ImportError:
    GroupSyncRead = None
    PacketHandler = None
    PortHandler = None

try:
    import serial
except ImportError as exc:
    raise ImportError(
        "pyserial is required. Install it with `pip install pyserial` before "
        "running ur5e_gello_state_publisher."
    ) from exc

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None


class STS3215UR5eReader:
    """Reads STS3215 positions and converts them to UR5e-style joint radians."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        offset_file: str,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.offset_file = self._resolve_offset_file(offset_file)

        self.servo_ids = [1, 2, 3, 4, 5, 6, 7]
        self.present_position_addr = 56
        self.read_length = 2

        self.signs = {
            1: -1,
            2: -1,
            3: 1,
            4: -1,
            5: -1,
            6: -1,
            7: -1,
        }

        self.count_to_rad = 2 * math.pi / 4096
        self.joint_limits = {
            1: (-2 * math.pi, 2 * math.pi),
            2: (-2.1, 2.1),
            3: (-2.1, 2.1),
            4: (-math.pi, math.pi),
            5: (-math.pi, math.pi),
            6: (-2 * math.pi, 2 * math.pi),
            7: (-1.5, 1.5),
        }
        self.max_step_rad = 1.50

        with open(self.offset_file, "r", encoding="utf-8") as file:
            self.offsets = json.load(file)

        self.use_scservo_sdk = (
            GroupSyncRead is not None
            and PacketHandler is not None
            and PortHandler is not None
        )
        if not self.use_scservo_sdk:
            raise ImportError(
                "scservo_sdk is required for synchronized STS3215 reads. "
                "Install it with `python3 -m pip install scservo_sdk`."
            )

        self.port_handler = None
        self.packet_handler = None
        self.group_sync_read = None
        self.serial_port = None

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open STS3215 port: {self.port}")

        if not self.port_handler.setBaudRate(self.baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"Failed to set STS3215 baudrate: {self.baudrate}")

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.present_position_addr,
            self.read_length,
        )

        for servo_id in self.servo_ids:
            if not self.group_sync_read.addParam(servo_id):
                raise RuntimeError(f"Failed to add servo ID {servo_id} to sync read")

        time.sleep(0.2)

        self.previous_radians: Dict[int, Optional[float]] = {
            servo_id: None for servo_id in self.servo_ids
        }
        self.unclamped_radians: Dict[int, Optional[float]] = {
            servo_id: None for servo_id in self.servo_ids
        }
        self.previous_raw_positions: Dict[int, Optional[int]] = {
            servo_id: None for servo_id in self.servo_ids
        }
        self.latest_gripper_raw: Optional[int] = None

    def _resolve_offset_file(self, offset_file: str) -> Path:
        offset_path = Path(offset_file).expanduser()
        if offset_path.is_absolute() and offset_path.exists():
            return offset_path

        candidates = [
            Path.cwd() / offset_path,
            Path.cwd().parent / offset_path,
            Path.cwd().parent.parent / offset_path,
            Path.cwd() / "configs" / offset_path,
            Path.cwd().parent / "configs" / offset_path,
        ]

        for candidate in candidates:
            if candidate.exists():
                return candidate

        searched = "\n".join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(
            f"Could not find offset file '{offset_file}'. Searched:\n{searched}"
        )

    def sync_read_positions(self) -> Dict[int, Optional[int]]:
        self.group_sync_read.txRxPacket()

        positions: Dict[int, Optional[int]] = {}
        for servo_id in self.servo_ids:
            available = self.group_sync_read.isAvailable(
                servo_id,
                self.present_position_addr,
                self.read_length,
            )
            if available:
                positions[servo_id] = self.group_sync_read.getData(
                    servo_id,
                    self.present_position_addr,
                    self.read_length,
                )
            else:
                positions[servo_id] = None

        return positions

    def checksum(self, packet_body: List[int]) -> int:
        return (~sum(packet_body)) & 0xFF

    def make_read_packet(self, servo_id: int, address: int, read_length: int) -> bytes:
        instruction = 0x02
        length = 0x04
        body = [servo_id, length, instruction, address, read_length]
        return bytes([0xFF, 0xFF] + body + [self.checksum(body)])

    def read_position_with_serial(self, servo_id: int) -> Optional[int]:
        packet = self.make_read_packet(
            servo_id,
            self.present_position_addr,
            self.read_length,
        )

        self.serial_port.reset_input_buffer()
        self.serial_port.write(packet)
        time.sleep(0.002)

        response = self.serial_port.read(20)
        if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF:
            low = response[5]
            high = response[6]
            return low + (high << 8)

        return None

    def wrapped_delta(self, current: int, offset: int) -> int:
        return (current - offset + 2048) % 4096 - 2048

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def limit_step(self, servo_id: int, new_value: float) -> float:
        previous = self.previous_radians[servo_id]
        if previous is None:
            return new_value

        difference = new_value - previous
        if difference > self.max_step_rad:
            return previous + self.max_step_rad
        if difference < -self.max_step_rad:
            return previous - self.max_step_rad

        return new_value

    def position_to_radian(self, servo_id: int, current_pos: int) -> float:
        offset = self.offsets.get(str(servo_id))
        if offset is None:
            previous = self.previous_radians[servo_id]
            return 0.0 if previous is None else previous

        previous_raw = self.previous_raw_positions[servo_id]
        previous_unclamped_rad = self.unclamped_radians[servo_id]

        if previous_raw is None or previous_unclamped_rad is None:
            raw_delta = self.wrapped_delta(current_pos, offset)
            unclamped_radian = raw_delta * self.signs[servo_id] * self.count_to_rad
        else:
            raw_step = self.wrapped_delta(current_pos, previous_raw)
            unclamped_radian = (
                previous_unclamped_rad
                + raw_step * self.signs[servo_id] * self.count_to_rad
            )

        min_limit, max_limit = self.joint_limits[servo_id]
        radian = self.clamp(unclamped_radian, min_limit, max_limit)
        radian = self.limit_step(servo_id, radian)

        self.previous_raw_positions[servo_id] = current_pos
        self.unclamped_radians[servo_id] = unclamped_radian
        self.previous_radians[servo_id] = radian
        return radian

    def read_joints_and_gripper(self) -> Tuple[List[float], float]:
        joints: List[float] = []
        gripper = 0.0
        positions = self.sync_read_positions()

        for servo_id in self.servo_ids:
            current_pos = positions.get(servo_id)
            if current_pos is None:
                previous = self.previous_radians[servo_id]
                radian = 0.0 if previous is None else previous
            else:
                radian = self.position_to_radian(servo_id, current_pos)

            if servo_id <= 6:
                joints.append(radian)
            else:
                if current_pos is not None:
                    self.latest_gripper_raw = current_pos
                gripper = radian

        return joints, gripper

    def close(self) -> None:
        if self.group_sync_read is not None:
            self.group_sync_read.clearParam()
        if self.port_handler is not None:
            self.port_handler.closePort()
        if self.serial_port is not None:
            self.serial_port.close()


class UR5eGelloPublisher(Node):
    def __init__(self) -> None:
        super().__init__("ur5e_gello_publisher")

        self.declare_parameter("port", "COM3")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("offset_file", "servo_offsets.json")
        self.declare_parameter("control_mode", "ros2_position")
        self.declare_parameter("robot_ip", "192.168.0.119")
        self.declare_parameter("rtde_velocity", 0.5)
        self.declare_parameter("rtde_acceleration", 0.5)
        self.declare_parameter("rtde_dt", 1.0 / 500.0)
        self.declare_parameter("rtde_lookahead_time", 0.2)
        self.declare_parameter("rtde_gain", 100)
        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("frame_id", "base")
        self.declare_parameter("joint_state_topic", "gello/joint_states")
        self.declare_parameter(
            "trajectory_topic",
            "scaled_joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter(
            "position_command_topic",
            "forward_position_controller/commands",
        )
        self.declare_parameter("gripper_topic", "gello/gripper_position")
        self.declare_parameter(
            "gripper_command_topic",
            "onrobot/finger_width_controller/commands",
        )
        self.declare_parameter(
            "gripper_command_topic_alt",
            "finger_width_controller/commands",
        )
        self.declare_parameter(
            "gripper_trajectory_topic",
            "finger_width_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter("publish_trajectory", True)
        self.declare_parameter("publish_position_command", False)
        self.declare_parameter("publish_gripper_command", True)
        self.declare_parameter("publish_gripper_trajectory", True)
        self.declare_parameter("trajectory_time_from_start_sec", 0.08)
        self.declare_parameter("command_smoothing_alpha", 0.25)
        self.declare_parameter("max_joint_speed_rad_per_sec", 0.5)
        self.declare_parameter("max_joint_delta_rad_per_cycle", 0.0)
        self.declare_parameter("command_deadband_rad", 0.002)
        self.declare_parameter("gripper_joint_name", "finger_width")
        self.declare_parameter("gripper_min_rad", -1.5)
        self.declare_parameter("gripper_max_rad", 1.5)
        self.declare_parameter("gripper_min_width", 0.0)
        self.declare_parameter("gripper_max_width", 0.1)
        self.declare_parameter("gripper_min_raw", 3400)
        self.declare_parameter("gripper_max_raw", 3800)
        self.declare_parameter("gripper_command_deadband_m", 0.004)
        self.declare_parameter("gripper_smoothing_alpha", 0.45)
        self.declare_parameter("gripper_reversal_deadband_m", 0.008)
        self.declare_parameter("invert_gripper", False)
        self.declare_parameter("align_to_robot_on_start", True)
        self.declare_parameter("robot_joint_state_topic", "joint_states")
        self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter("joint_position_offsets", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_names = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != 6:
            raise ValueError("joint_names must contain exactly 6 UR5e joint names")

        self.joint_position_offsets = [
            float(value) for value in self.get_parameter("joint_position_offsets").value
        ]
        if len(self.joint_position_offsets) != 6:
            raise ValueError("joint_position_offsets must contain exactly 6 values")

        self.control_mode = str(self.get_parameter("control_mode").value)
        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.rtde_velocity = float(self.get_parameter("rtde_velocity").value)
        self.rtde_acceleration = float(self.get_parameter("rtde_acceleration").value)
        self.rtde_dt = float(self.get_parameter("rtde_dt").value)
        self.rtde_lookahead_time = float(
            self.get_parameter("rtde_lookahead_time").value
        )
        self.rtde_gain = int(self.get_parameter("rtde_gain").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_trajectory = bool(self.get_parameter("publish_trajectory").value)
        self.publish_position_command = bool(
            self.get_parameter("publish_position_command").value
        )
        self.publish_gripper_command = bool(
            self.get_parameter("publish_gripper_command").value
        )
        self.publish_gripper_trajectory = bool(
            self.get_parameter("publish_gripper_trajectory").value
        )
        self.trajectory_time_from_start_sec = float(
            self.get_parameter("trajectory_time_from_start_sec").value
        )
        self.command_smoothing_alpha = self.clamp(
            float(self.get_parameter("command_smoothing_alpha").value),
            0.0,
            1.0,
        )
        self.max_joint_speed_rad_per_sec = max(
            0.0,
            float(self.get_parameter("max_joint_speed_rad_per_sec").value),
        )
        self.max_joint_delta_rad_per_cycle = max(
            0.0,
            float(self.get_parameter("max_joint_delta_rad_per_cycle").value),
        )
        self.command_deadband_rad = max(
            0.0,
            float(self.get_parameter("command_deadband_rad").value),
        )
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.gripper_min_rad = float(self.get_parameter("gripper_min_rad").value)
        self.gripper_max_rad = float(self.get_parameter("gripper_max_rad").value)
        self.gripper_min_width = float(self.get_parameter("gripper_min_width").value)
        self.gripper_max_width = float(self.get_parameter("gripper_max_width").value)
        self.gripper_min_raw = int(self.get_parameter("gripper_min_raw").value)
        self.gripper_max_raw = int(self.get_parameter("gripper_max_raw").value)
        self.gripper_command_deadband_m = max(
            0.0,
            float(self.get_parameter("gripper_command_deadband_m").value),
        )
        self.gripper_smoothing_alpha = self.clamp(
            float(self.get_parameter("gripper_smoothing_alpha").value),
            0.0,
            1.0,
        )
        self.gripper_reversal_deadband_m = max(
            0.0,
            float(self.get_parameter("gripper_reversal_deadband_m").value),
        )
        self.invert_gripper = bool(self.get_parameter("invert_gripper").value)
        self.align_to_robot_on_start = bool(
            self.get_parameter("align_to_robot_on_start").value
        )
        self.alignment_offsets = [0.0] * 6
        self.alignment_ready = not self.align_to_robot_on_start
        self.latest_robot_joints: Optional[List[float]] = None
        self.smoothed_target_joints: Optional[List[float]] = None
        self.filtered_commanded_joints: Optional[List[float]] = None
        self.filtered_gripper_width: Optional[float] = None
        self.last_gripper_command_width: Optional[float] = None
        self.last_gripper_direction = 0
        self.last_command_time = self.get_clock().now()
        self.rtde_control_interface = None
        self.rtde_receive_interface = None

        if self.control_mode == "rtde_servoj":
            if rtde_control is None or rtde_receive is None:
                raise ImportError(
                    "ur_rtde is required for control_mode=rtde_servoj. "
                    "Install it with `python3 -m pip install ur-rtde`."
                )

            self.get_logger().info(
                f"Connecting directly to UR RTDE at {self.robot_ip} for servoJ control."
            )
            self.rtde_control_interface = rtde_control.RTDEControlInterface(
                self.robot_ip
            )
            self.rtde_receive_interface = rtde_receive.RTDEReceiveInterface(
                self.robot_ip
            )
            try:
                self.rtde_control_interface.endFreedriveMode()
            except Exception as exc:
                self.get_logger().warn(f"endFreedriveMode skipped: {exc}")

        self.reader = STS3215UR5eReader(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            offset_file=str(self.get_parameter("offset_file").value),
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            10,
        )
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("trajectory_topic").value),
            10,
        )
        self.position_command_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("position_command_topic").value),
            10,
        )
        self.gripper_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("gripper_topic").value),
            10,
        )
        self.gripper_command_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("gripper_command_topic").value),
            10,
        )
        self.gripper_command_alt_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("gripper_command_topic_alt").value),
            10,
        )
        self.gripper_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("gripper_trajectory_topic").value),
            10,
        )
        self.robot_joint_state_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("robot_joint_state_topic").value),
            self.robot_joint_state_callback,
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_state)

        self.get_logger().info(
            "Publishing STS3215 GELLO as UR5e joint states"
            f" at {publish_rate_hz:.1f} Hz."
        )
        self.get_logger().info(
            "STS3215 GroupSyncRead enabled for servo IDs "
            f"{self.reader.servo_ids} at {self.reader.baudrate} baud."
        )
        if self.control_mode == "rtde_servoj":
            self.get_logger().info(
                "Using RTDE servoJ control "
                f"(dt={self.rtde_dt:.4f}, lookahead={self.rtde_lookahead_time:.3f}, "
                f"gain={self.rtde_gain})."
            )

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def robot_joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(joint_name in positions for joint_name in self.joint_names):
            self.latest_robot_joints = [
                float(positions[joint_name]) for joint_name in self.joint_names
            ]

    def publish_state(self) -> None:
        if self.control_mode == "rtde_servoj":
            self.update_robot_joints_from_rtde()

        joints, gripper = self.reader.read_joints_and_gripper()

        base_joints = [
            joint + offset
            for joint, offset in zip(joints, self.joint_position_offsets)
        ]

        if not self.alignment_ready:
            if self.latest_robot_joints is None:
                self.get_logger().warn(
                    "Waiting for robot /joint_states before sending GELLO commands.",
                    throttle_duration_sec=2.0,
                )
                return

            self.alignment_offsets = [
                robot_joint - gello_joint
                for robot_joint, gello_joint in zip(self.latest_robot_joints, base_joints)
            ]
            self.alignment_ready = True
            self.smoothed_target_joints = list(self.latest_robot_joints)
            self.filtered_commanded_joints = list(self.latest_robot_joints)
            self.last_command_time = self.get_clock().now()
            self.get_logger().info(
                "Aligned GELLO startup command to current robot joint state. "
                f"Alignment offsets: {[round(value, 4) for value in self.alignment_offsets]}"
            )

        target_joints = [
            joint + offset
            for joint, offset in zip(base_joints, self.alignment_offsets)
        ]
        commanded_joints = self.smooth_and_limit_command(target_joints)

        now = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.header.frame_id = self.frame_id
        joint_state.name = self.joint_names
        joint_state.position = commanded_joints
        self.joint_state_publisher.publish(joint_state)

        if self.control_mode == "rtde_servoj":
            self.send_rtde_servoj(commanded_joints)

        if self.publish_position_command:
            position_command = Float64MultiArray()
            position_command.data = commanded_joints
            self.position_command_publisher.publish(position_command)

        gripper_msg = Float32()
        gripper_msg.data = float(gripper)
        self.gripper_publisher.publish(gripper_msg)
        gripper_width = self.filter_gripper_width(self.gripper_to_width(gripper))

        if self.publish_gripper_command:
            gripper_command = Float64MultiArray()
            gripper_command.data = [gripper_width]
            self.gripper_command_publisher.publish(gripper_command)
            self.gripper_command_alt_publisher.publish(gripper_command)

        if self.publish_trajectory:
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = commanded_joints
            seconds = int(self.trajectory_time_from_start_sec)
            nanoseconds = int((self.trajectory_time_from_start_sec - seconds) * 1e9)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            trajectory.points = [point]

            self.trajectory_publisher.publish(trajectory)

        if self.publish_gripper_trajectory:
            gripper_trajectory = JointTrajectory()
            gripper_trajectory.joint_names = [self.gripper_joint_name]

            point = JointTrajectoryPoint()
            point.positions = [gripper_width]
            seconds = int(self.trajectory_time_from_start_sec)
            nanoseconds = int((self.trajectory_time_from_start_sec - seconds) * 1e9)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            gripper_trajectory.points = [point]

            self.gripper_trajectory_publisher.publish(gripper_trajectory)

    def smooth_and_limit_command(self, target_joints: List[float]) -> List[float]:
        now = self.get_clock().now()
        dt = max((now - self.last_command_time).nanoseconds * 1e-9, 1e-3)
        self.last_command_time = now

        if self.filtered_commanded_joints is None or self.smoothed_target_joints is None:
            self.smoothed_target_joints = list(target_joints)
            self.filtered_commanded_joints = list(target_joints)
            return list(target_joints)

        if self.max_joint_delta_rad_per_cycle > 0.0:
            max_step = self.max_joint_delta_rad_per_cycle
        else:
            max_step = self.max_joint_speed_rad_per_sec * dt

        reference_joints = (
            self.latest_robot_joints
            if self.latest_robot_joints is not None
            else self.filtered_commanded_joints
        )
        next_smoothed_targets: List[float] = []
        deltas: List[float] = []

        for reference, smoothed_target, target in zip(
            reference_joints,
            self.smoothed_target_joints,
            target_joints,
        ):
            if abs(target - reference) < self.command_deadband_rad:
                target = reference

            smoothed_target = (
                (1.0 - self.command_smoothing_alpha) * smoothed_target
                + self.command_smoothing_alpha * target
            )
            delta = smoothed_target - reference
            if abs(delta) < self.command_deadband_rad:
                delta = 0.0

            next_smoothed_targets.append(smoothed_target)
            deltas.append(delta)

        max_delta = max((abs(delta) for delta in deltas), default=0.0)
        if max_step > 0.0 and max_delta > max_step:
            scale = max_step / max_delta
            deltas = [delta * scale for delta in deltas]

        next_joints = [
            reference + delta
            for reference, delta in zip(reference_joints, deltas)
        ]

        self.smoothed_target_joints = next_smoothed_targets
        self.filtered_commanded_joints = next_joints
        return next_joints

    def update_robot_joints_from_rtde(self) -> None:
        if self.rtde_receive_interface is None:
            return

        try:
            joints = self.rtde_receive_interface.getActualQ()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read RTDE joint state: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        if joints is not None and len(joints) >= 6:
            self.latest_robot_joints = [float(value) for value in joints[:6]]

    def send_rtde_servoj(self, commanded_joints: List[float]) -> None:
        if self.rtde_control_interface is None:
            return

        try:
            period_start = self.rtde_control_interface.initPeriod()
            self.rtde_control_interface.servoJ(
                commanded_joints[:6],
                self.rtde_velocity,
                self.rtde_acceleration,
                self.rtde_dt,
                self.rtde_lookahead_time,
                self.rtde_gain,
            )
            self.rtde_control_interface.waitPeriod(period_start)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send RTDE servoJ command: {exc}",
                throttle_duration_sec=1.0,
            )

    def gripper_to_width(self, gripper_rad: float) -> float:
        gripper_raw = self.reader.latest_gripper_raw
        if gripper_raw is not None and self.gripper_max_raw != self.gripper_min_raw:
            normalized = (gripper_raw - self.gripper_min_raw) / (
                self.gripper_max_raw - self.gripper_min_raw
            )
        elif self.gripper_max_rad == self.gripper_min_rad:
            normalized = 0.0
        else:
            normalized = (gripper_rad - self.gripper_min_rad) / (
                self.gripper_max_rad - self.gripper_min_rad
            )

        normalized = max(0.0, min(normalized, 1.0))
        if self.invert_gripper:
            normalized = 1.0 - normalized

        return self.gripper_min_width + normalized * (
            self.gripper_max_width - self.gripper_min_width
        )

    def filter_gripper_width(self, width: float) -> float:
        width = self.clamp(width, self.gripper_min_width, self.gripper_max_width)
        if self.filtered_gripper_width is None:
            self.filtered_gripper_width = width
            self.last_gripper_command_width = width
            return width

        self.filtered_gripper_width = (
            (1.0 - self.gripper_smoothing_alpha) * self.filtered_gripper_width
            + self.gripper_smoothing_alpha * width
        )

        if self.last_gripper_command_width is None:
            self.last_gripper_command_width = self.filtered_gripper_width
            return self.filtered_gripper_width

        delta = self.filtered_gripper_width - self.last_gripper_command_width
        if abs(delta) < self.gripper_command_deadband_m:
            return self.last_gripper_command_width

        direction = 1 if delta > 0.0 else -1
        if (
            self.last_gripper_direction != 0
            and direction != self.last_gripper_direction
            and abs(delta) < self.gripper_reversal_deadband_m
        ):
            return self.last_gripper_command_width

        self.last_gripper_direction = direction
        self.last_gripper_command_width = self.filtered_gripper_width
        return self.filtered_gripper_width

    def destroy_node(self) -> None:
        if self.rtde_control_interface is not None:
            try:
                self.rtde_control_interface.servoStop()
            except Exception:
                pass
        self.reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = UR5eGelloPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
