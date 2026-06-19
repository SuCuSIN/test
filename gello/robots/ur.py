from typing import Dict

import numpy as np

from gello.robots.robot import Robot


class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = True):
        import rtde_control
        import rtde_receive

        print("Connecting to UR robot:", robot_ip)

        try:
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
        except Exception as e:
            print("Failed to create RTDEControlInterface.")
            print("Robot IP:", robot_ip)
            print("Error:", e)
            raise

        try:
            self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        except Exception as e:
            print("Failed to create RTDEReceiveInterface.")
            print("Robot IP:", robot_ip)
            print("Error:", e)
            raise

        self.gripper = None
        self._use_gripper = False
        self._free_drive = False

        try:
            self.robot.endFreedriveMode()
        except Exception as e:
            print("endFreedriveMode skipped:", e)

        print("UR robot connected.")

    def num_dofs(self) -> int:
        return 6

    def get_joint_state(self) -> np.ndarray:
        robot_joints = self.r_inter.getActualQ()
        return np.array(robot_joints)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        velocity = 0.5
        acceleration = 0.5
        dt = 1.0 / 500
        lookahead_time = 0.2
        gain = 100

        robot_joints = joint_state[:6]

        t_start = self.robot.initPeriod()
        self.robot.servoJ(
            robot_joints,
            velocity,
            acceleration,
            dt,
            lookahead_time,
            gain,
        )
        self.robot.waitPeriod(t_start)

    def freedrive_enabled(self) -> bool:
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([0.0])

        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot_ip = "192.168.1.11"
    ur = URRobot(robot_ip, no_gripper=True)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()