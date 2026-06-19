import numpy as np
import json
import math
import time
from pathlib import Path

# import serial
from scservo_sdk import PortHandler, PacketHandler, GroupSyncRead


class STS3215UR5eAgent:
    """
    STS3215 + URT-2 based UR5e-like passive controller agent.

    Servo mapping:
        ID 1 = Base
        ID 2 = Shoulder
        ID 3 = Elbow
        ID 4 = Wrist 1
        ID 5 = Wrist 2
        ID 6 = Wrist 3
        ID 7 = Gripper
    """

    def __init__(
        self,
        port="COM3",
        baudrate=1000000,
        offset_file="servo_offsets.json",
    ):
        self.port = port
        self.baudrate = baudrate

        # Find offset file from project root or configs folder
        project_root = Path(__file__).resolve().parents[2]
        offset_path_1 = project_root / offset_file
        offset_path_2 = project_root / "configs" / offset_file

        if offset_path_1.exists():
            self.offset_file = offset_path_1
        elif offset_path_2.exists():
            self.offset_file = offset_path_2
        else:
            raise FileNotFoundError(
                f"Could not find {offset_file}. "
                f"Place it in project root or configs folder."
            )

        self.servo_ids = [1, 2, 3, 4, 5, 6, 7]

        self.present_position_addr = 56
        self.read_length = 2

        # Direction signs confirmed by testing: ++-++-+
        self.signs = {
            1: -1,
            2: -1,
            3: 1,
            4: -1,
            5: -1,
            6: -1,
            7: -1,
        }

        self.joint_names = {
            1: "Base",
            2: "Shoulder",
            3: "Elbow",
            4: "Wrist 1",
            5: "Wrist 2",
            6: "Wrist 3",
            7: "Gripper",
        }

        # STS3215 raw count conversion
        # Assumption: 0-4095 counts = 360 degrees = 2*pi radians
        self.count_to_rad = 2 * math.pi / 4096
        self.count_to_deg = 360 / 4096

        # Safety limits in radians
        self.joint_limits = {
            1: (-2 * math.pi, 2 * math.pi),  # Base: allow wider rotation
            2: (-2.1, 2.1),
            3: (-2.1, 2.1),
            4: (-math.pi, math.pi),
            5: (-math.pi, math.pi),
            6: (-2 * math.pi, 2 * math.pi),  # Wrist 3
            7: (-1.5, 1.5),
        }

        # Limit sudden jump per cycle
        self.max_step_rad = 1.50

        with open(self.offset_file, "r") as file:
            self.offsets = json.load(file)

        self.protocol_end = 0

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(self.protocol_end)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {self.port}")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate: {self.baudrate}")

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.present_position_addr,
            self.read_length,
        )

        for servo_id in self.servo_ids:
            ok = self.group_sync_read.addParam(servo_id)
            if not ok:
                print(f"Warning: failed to add servo ID {servo_id} to sync read")

        time.sleep(0.2)

        self.previous_radians = {
            1: None,
            2: None,
            3: None,
            4: None,
            5: None,
            6: None,
            7: None,
        }

        self.previous_raw_positions = {
            1: None,
            2: None,
            3: None,
            4: None,
            5: None,
            6: None,
            7: None,
        }

    def checksum(self, packet_body):
        return (~sum(packet_body)) & 0xFF

    def make_read_packet(self, servo_id, address, read_length):
        instruction = 0x02
        length = 0x04
        body = [servo_id, length, instruction, address, read_length]
        return bytes([0xFF, 0xFF] + body + [self.checksum(body)])

    # def read_position(self, servo_id):
    #     packet = self.make_read_packet(
    #         servo_id,
    #         self.present_position_addr,
    #         self.read_length,
    #     )

    #     self.ser.reset_input_buffer()
    #     self.ser.write(packet)
    #     time.sleep(0.002)

    #     response = self.ser.read(20)

    #     if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF:
    #         low = response[5]
    #         high = response[6]
    #         return low + (high << 8)

    #     return None
    
    def sync_read_positions(self, servo_ids=None):
        """
        Sync read present position from multiple STS3215 servos.
        Returns:
            {servo_id: raw_position}
        """
        if servo_ids is None:
            servo_ids = self.servo_ids

        result = self.group_sync_read.txRxPacket()

        positions = {}

        for servo_id in servo_ids:
            available = self.group_sync_read.isAvailable(
                servo_id,
                self.present_position_addr,
                self.read_length,
            )

            if available:
                pos = self.group_sync_read.getData(
                    servo_id,
                    self.present_position_addr,
                    self.read_length,
                )
                positions[servo_id] = pos
            else:
                positions[servo_id] = None

        return positions

    def wrapped_delta(self, current, offset):
        return (current - offset + 2048) % 4096 - 2048

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def limit_step(self, servo_id, new_value):
        previous = self.previous_radians[servo_id]

        # First reading: accept actual value directly
        if previous is None:
            return new_value

        difference = new_value - previous

        if difference > self.max_step_rad:
            return previous + self.max_step_rad
        if difference < -self.max_step_rad:
            return previous - self.max_step_rad

        return new_value

    def position_to_radian(self, servo_id, current_pos):
        offset = self.offsets.get(str(servo_id))

        if offset is None:
            previous = self.previous_radians[servo_id]
            return 0.0 if previous is None else previous

        previous_raw = self.previous_raw_positions[servo_id]
        previous_rad = self.previous_radians[servo_id]

        # First reading: calculate from offset
        if previous_raw is None or previous_rad is None:
            delta = self.wrapped_delta(current_pos, offset)
            radian = delta * self.signs[servo_id] * self.count_to_rad

            min_limit, max_limit = self.joint_limits[servo_id]
            radian = self.clamp(radian, min_limit, max_limit)

            self.previous_raw_positions[servo_id] = current_pos
            self.previous_radians[servo_id] = radian
            return radian

        # Continuous tracking after first reading
        raw_step = self.wrapped_delta(current_pos, previous_raw)
        radian = previous_rad + raw_step * self.signs[servo_id] * self.count_to_rad

        min_limit, max_limit = self.joint_limits[servo_id]
        radian = self.clamp(radian, min_limit, max_limit)

        # Optional safety step limit
        radian = self.limit_step(servo_id, radian)

        self.previous_raw_positions[servo_id] = current_pos
        self.previous_radians[servo_id] = radian
        return radian

    def read_joints_and_gripper(self):
        """
        Read all STS3215 servos using Sync Read and return:
            joints: [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]
            gripper: ID 7 value

        Unit: radians
        """
        joints = []
        gripper = None

        positions = self.sync_read_positions(self.servo_ids)

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
                gripper = radian

        return joints, gripper

    def act(self, obs=None):
        """
        Return 7 controller values for GELLO run_env:
        [base, shoulder, elbow, wrist_1, wrist_2, wrist_3, gripper]
        """
        joints, gripper = self.read_joints_and_gripper()

        if gripper is None:
            previous = self.previous_radians[7]
            gripper = 0.0 if previous is None else previous

        action = joints + [gripper]

        return np.array(action, dtype=np.float32)
    def get_action_with_gripper(self):
        """
        Return:
            joints: [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]
            gripper: gripper value
        """
        return self.read_joints_and_gripper()

    def close(self):
        self.group_sync_read.clearParam()
        self.port_handler.closePort()
