import os
import signal
import subprocess
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class AutoDemoRecorder(Node):
    def __init__(self) -> None:
        super().__init__("auto_demo_recorder")

        self.declare_parameter("output_dir", "demos")
        self.declare_parameter("prefix", "demo")
        self.declare_parameter("start_index", 1)
        self.declare_parameter("home_joint_topic", "gello/joint_states")
        self.declare_parameter(
            "gripper_command_topic",
            "onrobot/finger_width_controller/commands",
        )
        self.declare_parameter("home_tolerance_rad", 0.12)
        self.declare_parameter("close_width_m", 0.045)
        self.declare_parameter("open_width_m", 0.10)
        self.declare_parameter("cooldown_sec", 1.5)
        self.declare_parameter(
            "record_topics",
            [
                "/gello/joint_states",
                "/joint_states",
                "/gello/gripper_position",
                "/onrobot/finger_width_controller/commands",
            ],
        )

        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.prefix = str(self.get_parameter("prefix").value)
        self.next_index = int(self.get_parameter("start_index").value)
        self.home_tolerance_rad = float(
            self.get_parameter("home_tolerance_rad").value
        )
        self.close_width_m = float(self.get_parameter("close_width_m").value)
        self.open_width_m = float(self.get_parameter("open_width_m").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.record_topics = [
            str(topic) for topic in self.get_parameter("record_topics").value
        ]

        self.home_positions: Optional[Dict[str, float]] = None
        self.latest_home_error = float("inf")
        self.armed_for_open = False
        self.cooldown_until = 0.0
        self.record_process: Optional[subprocess.Popen] = None

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.next_index = self.find_next_available_index(self.next_index)

        self.create_subscription(
            JointState,
            str(self.get_parameter("home_joint_topic").value),
            self.joint_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("gripper_command_topic").value),
            self.gripper_callback,
            10,
        )

        self.start_recording()
        self.get_logger().info(
            "Auto demo recorder ready. Move to the startup home pose, "
            "then close and open the gripper to advance to the next demo."
        )

    def find_next_available_index(self, start_index: int) -> int:
        index = max(1, start_index)
        while self.demo_path(index).exists():
            index += 1
        return index

    def demo_path(self, index: int) -> Path:
        return self.output_dir / f"{self.prefix}_{index:03d}"

    def start_recording(self) -> None:
        demo_path = self.demo_path(self.next_index)
        command = ["ros2", "bag", "record", *self.record_topics, "-o", str(demo_path)]
        self.record_process = subprocess.Popen(command, start_new_session=True)
        self.get_logger().info(f"Recording {demo_path}")

    def stop_recording(self) -> None:
        if self.record_process is None:
            return
        if self.record_process.poll() is None:
            os.killpg(self.record_process.pid, signal.SIGINT)
            try:
                self.record_process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.record_process.pid, signal.SIGTERM)
                self.record_process.wait(timeout=3.0)
        self.record_process = None

    def rotate_recording(self) -> None:
        self.get_logger().info(
            f"Finish trigger detected near home. Closing {self.demo_path(self.next_index)}"
        )
        self.stop_recording()
        self.next_index = self.find_next_available_index(self.next_index + 1)
        self.start_recording()
        self.armed_for_open = False
        self.cooldown_until = self.get_clock().now().nanoseconds * 1e-9 + self.cooldown_sec

    def joint_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not positions:
            return
        if self.home_positions is None:
            self.home_positions = positions
            self.latest_home_error = 0.0
            self.get_logger().info(
                "Captured startup GELLO pose as demo split home pose."
            )
            return

        common_names = [
            name for name in self.home_positions.keys() if name in positions
        ]
        if not common_names:
            return

        self.latest_home_error = max(
            abs(positions[name] - self.home_positions[name])
            for name in common_names
        )

    def gripper_callback(self, msg: Float64MultiArray) -> None:
        if not msg.data or self.home_positions is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec < self.cooldown_until:
            return

        if self.latest_home_error > self.home_tolerance_rad:
            return

        width = float(msg.data[0])
        if not self.armed_for_open and width <= self.close_width_m:
            self.armed_for_open = True
            self.get_logger().info("Close gesture detected near home. Open to split.")
            return

        if self.armed_for_open and width >= self.open_width_m:
            self.rotate_recording()

    def destroy_node(self) -> bool:
        self.stop_recording()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AutoDemoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
