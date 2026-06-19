import serial
import time
import json
import math


class STS3215UR5eAgent:
    def __init__(
        self,
        port="COM3",
        baudrate=1000000,
        offset_file="servo_offsets.json",
    ):
        self.port = port
        self.baudrate = baudrate
        self.offset_file = offset_file

        # Servo ID mapping
        # ID 1 = Base
        # ID 2 = Shoulder
        # ID 3 = Elbow
        # ID 4 = Wrist 1
        # ID 5 = Wrist 2
        # ID 6 = Wrist 3
        # ID 7 = Gripper
        self.servo_ids = [1, 2, 3, 4, 5, 6, 7]

        self.present_position_addr = 56
        self.read_length = 2

        # Direction signs confirmed by testing: ++-++-+
        self.signs = {
            1: 1,
            2: 1,
            3: -1,
            4: 1,
            5: 1,
            6: -1,
            7: 1,
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
        # Assumption: 0–4095 counts = 360 degrees = 2π radians
        self.count_to_rad = 2 * math.pi / 4096
        self.count_to_deg = 360 / 4096

        # Joint limits in radians
        # These are safety limits for testing, not exact UR5e factory limits.
        self.joint_limits = {
            1: (-math.pi, math.pi),          # Base
            2: (-2.1, 2.1),                  # Shoulder
            3: (-2.1, 2.1),                  # Elbow
            4: (-math.pi, math.pi),          # Wrist 1
            5: (-math.pi, math.pi),          # Wrist 2
            6: (-math.pi, math.pi),          # Wrist 3
            7: (-1.5, 1.5),                  # Gripper / extra joint
        }

        # Maximum allowed change per read cycle in radians.
        # This reduces sudden jump/noise.
        self.max_step_rad = 0.25

        with open(self.offset_file, "r") as file:
            self.offsets = json.load(file)

        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        time.sleep(0.2)

        # Store previous values for safety filtering
        self.previous_radians = {
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
            7: 0.0,
        }

    def checksum(self, packet_body):
        return (~sum(packet_body)) & 0xFF

    def make_read_packet(self, servo_id, address, read_length):
        instruction = 0x02  # READ
        length = 0x04
        body = [servo_id, length, instruction, address, read_length]
        return bytes([0xFF, 0xFF] + body + [self.checksum(body)])

    def read_position(self, servo_id):
        packet = self.make_read_packet(
            servo_id,
            self.present_position_addr,
            self.read_length,
        )

        self.ser.reset_input_buffer()
        self.ser.write(packet)
        time.sleep(0.015)

        response = self.ser.read(20)

        if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF:
            low = response[5]
            high = response[6]
            position = low + (high << 8)
            return position

        return None

    def wrapped_delta(self, current, offset):
        """
        Prevent sudden jump when raw position crosses 0/4095 boundary.
        """
        return (current - offset + 2048) % 4096 - 2048

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def limit_step(self, servo_id, new_value):
        """
        Limits sudden changes between current and previous joint angle.
        If the value jumps too much, only allow a small change.
        """
        previous = self.previous_radians[servo_id]
        difference = new_value - previous

        if difference > self.max_step_rad:
            new_value = previous + self.max_step_rad
        elif difference < -self.max_step_rad:
            new_value = previous - self.max_step_rad

        return new_value

    def position_to_radian(self, servo_id, current_pos):
        offset = self.offsets.get(str(servo_id))

        if offset is None:
            return self.previous_radians[servo_id]

        delta = self.wrapped_delta(current_pos, offset)
        corrected = delta * self.signs[servo_id]
        radian = corrected * self.count_to_rad

        # Apply joint limits
        min_limit, max_limit = self.joint_limits[servo_id]
        radian = self.clamp(radian, min_limit, max_limit)

        # Apply sudden jump filtering
        radian = self.limit_step(servo_id, radian)

        # Save as previous value
        self.previous_radians[servo_id] = radian

        return radian

    def position_to_degree(self, servo_id, current_pos):
        radian = self.position_to_radian(servo_id, current_pos)
        return radian * 180 / math.pi

    def act(self):
        """
        Main function for GELLO-style usage.

        Returns:
            ur5e_joints:
                [base, shoulder, elbow, wrist_1, wrist_2, wrist_3]
                Unit: radians

            gripper:
                gripper value in radians
        """
        ur5e_joints = []
        gripper = None

        for servo_id in self.servo_ids:
            current_pos = self.read_position(servo_id)

            # If reading fails, use previous safe value instead of returning None.
            if current_pos is None:
                radian = self.previous_radians[servo_id]
            else:
                radian = self.position_to_radian(servo_id, current_pos)

            if servo_id <= 6:
                ur5e_joints.append(radian)
            else:
                gripper = radian

        return ur5e_joints, gripper

    def get_action(self):
        """
        Returns only the 6 UR5e joint radians.
        This is useful for GELLO/simulation connection.
        """
        joints, _ = self.act()
        return joints

    def get_action_with_gripper(self):
        """
        Returns both 6 UR5e joint radians and gripper value.
        """
        return self.act()

    def read_debug(self):
        """
        Returns raw position, degree, and radian values for debugging.
        """
        debug_data = {}

        for servo_id in self.servo_ids:
            current_pos = self.read_position(servo_id)

            if current_pos is None:
                radian = self.previous_radians[servo_id]
                degree = radian * 180 / math.pi

                debug_data[servo_id] = {
                    "name": self.joint_names[servo_id],
                    "raw": None,
                    "degree": degree,
                    "radian": radian,
                    "status": "read failed, using previous value",
                }
                continue

            radian = self.position_to_radian(servo_id, current_pos)
            degree = radian * 180 / math.pi

            debug_data[servo_id] = {
                "name": self.joint_names[servo_id],
                "raw": current_pos,
                "degree": degree,
                "radian": radian,
                "status": "ok",
            }

        return debug_data

    def close(self):
        self.ser.close()


def main():
    agent = STS3215UR5eAgent()

    print("UR5e Agent started.")
    print("Safety features enabled:")
    print("- Joint angle limits")
    print("- Previous value fallback")
    print("- Sudden jump filtering")
    print("- get_action() support")
    print("\nPress Ctrl + C to stop.\n")

    try:
        while True:
            joints, gripper = agent.get_action_with_gripper()
            debug_data = agent.read_debug()

            print("UR5e joints rad:")
            print([round(value, 4) for value in joints])

            print("UR5e joints deg:")
            degrees = []
            for servo_id in range(1, 7):
                degree = debug_data[servo_id]["degree"]
                degrees.append(round(degree, 2))
            print(degrees)

            print("Gripper rad:")
            print(round(gripper, 4) if gripper is not None else None)
            print("Gripper deg:")
            gripper_degree = debug_data[7]["degree"]
            print(round(gripper_degree, 2) if gripper_degree is not None else None)

            print("Status:")
            for servo_id in range(1, 8):
                name = debug_data[servo_id]["name"]
                status = debug_data[servo_id]["status"]
                print(f"ID {servo_id} {name}: {status}")

            print("-" * 60)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        agent.close()


if __name__ == "__main__":
    main()