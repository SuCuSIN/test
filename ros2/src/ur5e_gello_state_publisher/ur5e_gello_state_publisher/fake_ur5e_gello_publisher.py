import math
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FakeUR5eGelloPublisher(Node):
    """Publishes smooth fake GELLO commands for testing ROS 2 wiring."""

    def __init__(self) -> None:
        super().__init__("fake_ur5e_gello_publisher")

        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("frame_id", "base")
        self.declare_parameter("joint_state_topic", "gello/joint_states")
        self.declare_parameter(
            "trajectory_topic",
            "joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter("gripper_topic", "gello/gripper_position")
        self.declare_parameter("publish_trajectory", True)
        self.declare_parameter("trajectory_time_from_start_sec", 0.12)
        self.declare_parameter("amplitude_scale", 0.35)
        self.declare_parameter("motion_period_sec", 8.0)
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

        self.joint_names = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != 6:
            raise ValueError("joint_names must contain exactly 6 UR5e joint names")

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_trajectory = bool(self.get_parameter("publish_trajectory").value)
        self.trajectory_time_from_start_sec = float(
            self.get_parameter("trajectory_time_from_start_sec").value
        )
        self.amplitude_scale = float(self.get_parameter("amplitude_scale").value)
        self.motion_period_sec = float(self.get_parameter("motion_period_sec").value)

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
        self.gripper_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("gripper_topic").value),
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_state)

        self.get_logger().info(
            "Publishing fake UR5e GELLO commands"
            f" at {publish_rate_hz:.1f} Hz."
        )

    def _fake_joints(self, elapsed_sec: float) -> List[float]:
        phase = 2.0 * math.pi * elapsed_sec / self.motion_period_sec
        amplitudes = [
            0.45,
            0.35,
            0.45,
            0.55,
            0.40,
            0.60,
        ]
        phase_offsets = [0.0, 0.8, 1.6, 2.2, 3.0, 3.8]

        return [
            self.amplitude_scale * amp * math.sin(phase + offset)
            for amp, offset in zip(amplitudes, phase_offsets)
        ]

    def publish_state(self) -> None:
        now_clock = self.get_clock().now()
        elapsed_sec = (now_clock - self.start_time).nanoseconds / 1e9
        joints = self._fake_joints(elapsed_sec)
        gripper = 0.5 + 0.5 * math.sin(2.0 * math.pi * elapsed_sec / 4.0)

        now = now_clock.to_msg()

        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.header.frame_id = self.frame_id
        joint_state.name = self.joint_names
        joint_state.position = joints
        self.joint_state_publisher.publish(joint_state)

        gripper_msg = Float32()
        gripper_msg.data = float(gripper)
        self.gripper_publisher.publish(gripper_msg)

        if self.publish_trajectory:
            trajectory = JointTrajectory()
            trajectory.header.stamp = now
            trajectory.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = joints
            seconds = int(self.trajectory_time_from_start_sec)
            nanoseconds = int((self.trajectory_time_from_start_sec - seconds) * 1e9)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            trajectory.points = [point]

            self.trajectory_publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = FakeUR5eGelloPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
