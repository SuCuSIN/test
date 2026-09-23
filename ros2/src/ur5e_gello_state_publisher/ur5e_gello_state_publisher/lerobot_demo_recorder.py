from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
except ImportError:
    pa = None
    pq = None


class LeRobotDemoRecorder(Node):
    """Record UR5e + GELLO + RG6 demos as an OpenPI-friendly LeRobot v3 dataset."""

    def __init__(self) -> None:
        super().__init__("lerobot_demo_recorder")

        self.declare_parameter("repo_id", "local/ur5e_gello_rg6")
        self.declare_parameter("root", "lerobot_demos/ur5e_gello_rg6")
        self.declare_parameter("fps", 10)
        self.declare_parameter("task", "pick and place the object")
        self.declare_parameter("robot_type", "ur5e_gello_rg6")
        self.declare_parameter("image_feature_key", "observation.images.main")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("robot_joint_topic", "/joint_states")
        self.declare_parameter("gello_joint_topic", "/gello/joint_states")
        self.declare_parameter(
            "gripper_command_topic",
            "/onrobot/finger_width_controller/commands",
        )
        self.declare_parameter("image_height", 480)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("video_codec", "mp4v")
        self.declare_parameter("home_tolerance_rad", 0.12)
        self.declare_parameter("require_home_for_split", True)
        self.declare_parameter("home_settle_sec", 0.75)
        self.declare_parameter("split_cooldown_sec", 1.5)
        self.declare_parameter("auto_save_enabled", False)
        self.declare_parameter("save_partial_episode_on_shutdown", False)
        self.declare_parameter("min_episode_frames", 5)
        self.declare_parameter("save_episode_bundles", False)
        self.declare_parameter("episode_bundle_dir", "episode_bundles")

        if pa is None or pq is None:
            raise ImportError(
                "pyarrow is required for OpenPI/LeRobot parquet writing. "
                "Install it in WSL with `python3 -m pip install pyarrow`."
            )
        if cv2 is None:
            raise ImportError(
                "OpenCV is required for MP4 writing. Install it in WSL with "
                "`sudo apt install python3-opencv` or `python3 -m pip install opencv-python`."
            )

        self.repo_id = str(self.get_parameter("repo_id").value)
        self.root = Path(str(self.get_parameter("root").value))
        self.fps = int(self.get_parameter("fps").value)
        self.task = str(self.get_parameter("task").value)
        self.robot_type = str(self.get_parameter("robot_type").value)
        self.image_feature_key = str(self.get_parameter("image_feature_key").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.video_codec = str(self.get_parameter("video_codec").value)
        self.home_tolerance_rad = float(
            self.get_parameter("home_tolerance_rad").value
        )
        self.require_home_for_split = bool(
            self.get_parameter("require_home_for_split").value
        )
        self.home_settle_sec = float(self.get_parameter("home_settle_sec").value)
        self.split_cooldown_sec = float(
            self.get_parameter("split_cooldown_sec").value
        )
        self.auto_save_enabled = bool(
            self.get_parameter("auto_save_enabled").value
        )
        self.save_partial_episode_on_shutdown = bool(
            self.get_parameter("save_partial_episode_on_shutdown").value
        )
        self.min_episode_frames = int(self.get_parameter("min_episode_frames").value)
        self.save_episode_bundles = bool(
            self.get_parameter("save_episode_bundles").value
        )
        self.episode_bundle_dir = str(
            self.get_parameter("episode_bundle_dir").value
        )

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.motor_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
            "gripper_width",
        ]

        self.latest_robot_joints: Optional[np.ndarray] = None
        self.latest_gello_joints: Optional[np.ndarray] = None
        self.latest_gripper_width = 0.13
        self.latest_image: Optional[np.ndarray] = None
        self.home_gello_joints: Optional[np.ndarray] = None
        self.latest_home_error = float("inf")
        self.near_home_since: Optional[float] = None
        self.episode_left_home = False
        self.cooldown_until = 0.0
        self.last_status_log_sec = 0.0

        self.episode_images: List[np.ndarray] = []
        self.episode_states: List[np.ndarray] = []
        self.episode_actions: List[np.ndarray] = []
        self.episode_timestamps: List[float] = []
        self.saving_episode = False

        self.root = self.resolve_dataset_root(self.root)
        self.ensure_layout()
        self.total_frames, self.total_episodes = self.read_totals()

        self.create_subscription(
            JointState,
            str(self.get_parameter("robot_joint_topic").value),
            self.robot_joint_callback,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("gello_joint_topic").value),
            self.gello_joint_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("gripper_command_topic").value),
            self.gripper_callback,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self.image_callback,
            10,
        )
        self.create_subscription(Bool, "/save_demo", self.save_demo_callback, 10)
        self.create_service(Trigger, "save_episode", self.save_episode_service)
        self.create_timer(1.0 / self.fps, self.record_timer_callback)

        self.get_logger().info(
            f"Recording OpenPI-compatible LeRobot v3 dataset at {self.root}. "
            "Call the save_episode service to save the current recording."
        )
        if self.auto_save_enabled:
            self.get_logger().info(
                "Auto-save is enabled: returning near the startup pose will save "
                "each episode."
            )

    @property
    def data_path(self) -> Path:
        return self.data_path_for(self.root)

    @property
    def video_path(self) -> Path:
        return self.video_path_for(self.root)

    @property
    def tasks_path(self) -> Path:
        return self.tasks_path_for(self.root)

    @property
    def episodes_path(self) -> Path:
        return self.episodes_path_for(self.root)

    @property
    def info_path(self) -> Path:
        return self.info_path_for(self.root)

    @property
    def stats_path(self) -> Path:
        return self.stats_path_for(self.root)

    def data_path_for(self, root: Path) -> Path:
        return root / "data" / "chunk-000" / "file-000.parquet"

    def video_path_for(self, root: Path) -> Path:
        return (
            root
            / "videos"
            / self.image_feature_key
            / "chunk-000"
            / "file-000.mp4"
        )

    def episode_video_path_for(self, root: Path, episode_index: int) -> Path:
        return (
            root
            / "videos"
            / self.image_feature_key
            / "chunk-000"
            / f"episode_{episode_index:06d}.mp4"
        )

    def tasks_path_for(self, root: Path) -> Path:
        return root / "meta" / "tasks.parquet"

    def episodes_path_for(self, root: Path) -> Path:
        return root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"

    def info_path_for(self, root: Path) -> Path:
        return root / "meta" / "info.json"

    def stats_path_for(self, root: Path) -> Path:
        return root / "meta" / "stats.json"

    def resolve_dataset_root(self, requested_root: Path) -> Path:
        info_path = requested_root / "meta" / "info.json"
        if not info_path.exists():
            return requested_root

        try:
            info = json.loads(info_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return self.next_dataset_root(requested_root)

        if info.get("robot_type") != self.robot_type:
            return self.next_dataset_root(requested_root)

        existing_task = self.read_task_from_root(requested_root)
        if existing_task is not None and existing_task != self.task:
            return self.next_dataset_root(requested_root)

        video_path = info.get("video_path")
        if info.get("total_frames", 0) and not video_path:
            compatible_root = self.next_dataset_root(requested_root)
            self.get_logger().warn(
                f"Existing dataset at {requested_root} has no video_path; "
                f"starting compatible dataset at {compatible_root}."
            )
            return compatible_root
        if info.get("total_frames", 0) and "episode_index" not in str(video_path):
            compatible_root = self.next_dataset_root(requested_root)
            self.get_logger().warn(
                f"Existing dataset at {requested_root} uses a single shared video; "
                f"starting episode-video dataset at {compatible_root}."
            )
            return compatible_root

        return requested_root

    def read_task_from_root(self, root: Path) -> Optional[str]:
        tasks_path = root / "meta" / "tasks.parquet"
        if not tasks_path.exists():
            return None
        try:
            table = pq.read_table(tasks_path)
            if table.num_rows == 0 or "task" not in table.column_names:
                return None
            return str(table["task"][0].as_py())
        except Exception:
            return None

    def next_dataset_root(self, root: Path) -> Path:
        index = 2
        while True:
            candidate = root.with_name(f"{root.name}_{index:03d}")
            if not candidate.exists():
                return candidate
            index += 1

    def ensure_layout(self) -> None:
        for path in [
            self.data_path.parent,
            (self.root / "videos" / self.image_feature_key / "chunk-000"),
            self.episodes_path.parent,
            self.tasks_path.parent,
        ]:
            path.mkdir(parents=True, exist_ok=True)

        if not self.tasks_path.exists():
            self.write_tasks_table()
        if not self.info_path.exists():
            self.write_info(total_frames=0, total_episodes=0)
        if not self.stats_path.exists():
            self.write_stats(empty=True)

    def read_totals(self) -> Tuple[int, int]:
        if not self.info_path.exists():
            return 0, 0
        try:
            info = json.loads(self.info_path.read_text(encoding="utf-8"))
            return int(info.get("total_frames", 0)), int(info.get("total_episodes", 0))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return 0, 0

    def features(self) -> dict:
        return {
            "observation.state": {
                "dtype": "float32",
                "shape": [7],
                "names": {"motors": self.motor_names},
            },
            "action": {
                "dtype": "float32",
                "shape": [7],
                "names": {"motors": self.motor_names},
            },
            self.image_feature_key: {
                "dtype": "video",
                "shape": [self.image_height, self.image_width, 3],
                "names": ["height", "width", "channel"],
            },
            "joints": {
                "dtype": "float32",
                "shape": [6],
                "names": {"motors": self.motor_names[:6]},
            },
            "gripper": {
                "dtype": "float32",
                "shape": [1],
                "names": ["gripper_width"],
            },
            "actions": {
                "dtype": "float32",
                "shape": [7],
                "names": {"motors": self.motor_names},
            },
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
        }

    def ordered_joints(self, msg: JointState) -> Optional[np.ndarray]:
        positions: Dict[str, float] = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return None
        return np.asarray([positions[name] for name in self.joint_names], dtype=np.float32)

    def robot_joint_callback(self, msg: JointState) -> None:
        joints = self.ordered_joints(msg)
        if joints is not None:
            self.latest_robot_joints = joints

    def gello_joint_callback(self, msg: JointState) -> None:
        joints = self.ordered_joints(msg)
        if joints is None:
            return
        self.latest_gello_joints = joints
        if self.home_gello_joints is None:
            self.home_gello_joints = joints.copy()
            self.latest_home_error = 0.0
            self.get_logger().info("Captured startup GELLO pose as episode split home pose.")
        else:
            self.latest_home_error = float(np.max(np.abs(joints - self.home_gello_joints)))
        self.update_near_home_state()
        self.maybe_split_episode()

    def gripper_callback(self, msg: Float64MultiArray) -> None:
        if msg.data:
            self.latest_gripper_width = float(msg.data[0])

    def image_callback(self, msg: Image) -> None:
        try:
            image = self.image_msg_to_rgb(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return
        if image.shape[:2] != (self.image_height, self.image_width):
            self.get_logger().warn(
                f"Camera image shape {image.shape[:2]} does not match dataset shape "
                f"{(self.image_height, self.image_width)}.",
                throttle_duration_sec=2.0,
            )
            return
        self.latest_image = image

    def record_timer_callback(self) -> None:
        self.record_frame()

    def record_frame(self) -> None:
        if self.latest_robot_joints is None or self.latest_gello_joints is None:
            self.log_missing_inputs()
            return

        gripper = np.asarray([self.latest_gripper_width], dtype=np.float32)
        state = np.concatenate([self.latest_robot_joints, gripper]).astype(np.float32)
        action = np.concatenate([self.latest_gello_joints, gripper]).astype(np.float32)
        image = self.latest_image if self.latest_image is not None else self.blank_image()

        frame_index = len(self.episode_states)
        self.episode_states.append(state.copy())
        self.episode_actions.append(action.copy())
        self.episode_images.append(image.copy())
        self.episode_timestamps.append(float(frame_index) / float(self.fps))

    def log_missing_inputs(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_status_log_sec < 5.0:
            return
        self.last_status_log_sec = now_sec

        missing = []
        if self.latest_robot_joints is None:
            missing.append("robot /joint_states")
        if self.latest_gello_joints is None:
            missing.append("GELLO /gello/joint_states")
        self.get_logger().warn(
            "Not recording frames yet; waiting for " + ", ".join(missing) + "."
        )

    def update_near_home_state(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.latest_home_error <= self.home_tolerance_rad:
            if self.near_home_since is None:
                self.near_home_since = now_sec
        else:
            self.near_home_since = None
            self.episode_left_home = True

    def maybe_split_episode(self) -> None:
        if not self.auto_save_enabled:
            return
        if self.home_gello_joints is None:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec < self.cooldown_until:
            return
        if self.require_home_for_split and not self.is_home_ready(now_sec):
            return
        if self.episode_left_home and self.save_episode():
            self.get_logger().info("Auto-saved episode after returning to home pose.")
            self.cooldown_until = now_sec + self.split_cooldown_sec
            self.episode_left_home = False

    def is_home_ready(self, now_sec: float) -> bool:
        if self.latest_home_error > self.home_tolerance_rad:
            return False
        if self.near_home_since is None:
            return False
        return now_sec - self.near_home_since >= self.home_settle_sec

    def save_episode_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        response.success = self.save_episode()
        response.message = (
            "episode saved; start the next demo now"
            if response.success
            else "no frames recorded"
        )
        return response

    def save_demo_callback(self, message: Bool) -> None:
        if message.data:
            if self.save_episode():
                self.get_logger().info("Manual /save_demo request saved the episode.")

    def save_episode(self) -> bool:
        if self.saving_episode:
            self.get_logger().warn("Episode save is already in progress.")
            return False
        if len(self.episode_states) < self.min_episode_frames:
            self.get_logger().warn(
                "Save requested, but no complete episode has been recorded yet."
            )
            return False

        episode_index = self.total_episodes
        start_index = self.total_frames
        length = len(self.episode_states)
        staging_root = self.root / f".tmp_episode_{episode_index:06d}"
        backup_root = self.root / f".backup_episode_{episode_index:06d}"

        self.saving_episode = True
        self.get_logger().info(
            f"Staging episode {episode_index} with {length} frames..."
        )
        self.get_logger().warn(
            "Saving episode now. Please keep the robot and GELLO still until "
            "the save-complete message appears."
        )
        try:
            self.remove_tree(staging_root)
            self.remove_tree(backup_root)
            staged_relative_paths = self.write_staged_dataset(
                staging_root=staging_root,
                episode_index=episode_index,
                start_index=start_index,
                length=length,
            )

            staged_total_frames = self.total_frames + length
            staged_total_episodes = self.total_episodes + 1
            self.get_logger().info("Validating staged episode...")
            self.validate_dataset(
                root=staging_root,
                total_frames=staged_total_frames,
                total_episodes=staged_total_episodes,
                log_success=False,
            )

            self.get_logger().info(f"Committing episode {episode_index}...")
            self.commit_staged_dataset(
                staging_root,
                backup_root,
                extra_relative_paths=staged_relative_paths,
            )

            self.total_frames = staged_total_frames
            self.total_episodes = staged_total_episodes
            self.validate_dataset()
            if self.save_episode_bundles:
                try:
                    bundle_path = self.write_episode_bundle(
                        episode_index=episode_index,
                        start_index=start_index,
                        length=length,
                    )
                    self.get_logger().info(f"Saved episode bundle {bundle_path}.")
                except Exception as bundle_exc:
                    self.get_logger().warn(
                        f"Dataset was saved, but the per-demo bundle failed: {bundle_exc}"
                    )
            self.clear_episode_buffers()
            self.remove_tree(backup_root)
            self.get_logger().info(
                f"Episode {episode_index} save complete. You can start the next "
                "demo now."
            )
            return True
        except BaseException as exc:
            self.rollback_staged_save(
                staging_root,
                backup_root,
                extra_relative_paths=locals().get("staged_relative_paths", []),
            )
            self.total_frames = start_index
            self.total_episodes = episode_index
            self.get_logger().warn(
                f"Save interrupted; rolled back episode {episode_index}. "
                f"Dataset remains at {self.total_episodes} episodes, "
                f"{self.total_frames} frames. Reason: {exc}"
            )
            if isinstance(exc, KeyboardInterrupt):
                raise
            return False
        finally:
            self.saving_episode = False

    def write_staged_dataset(
        self,
        staging_root: Path,
        episode_index: int,
        start_index: int,
        length: int,
    ) -> List[Path]:
        self.get_logger().info("Writing temporary parquet/video...")
        existing_data = self.read_table_if_exists(self.data_path)
        existing_episodes = self.read_table_if_exists(self.episodes_path)

        new_data = self.build_episode_table(
            episode_index=episode_index,
            start_index=start_index,
        )
        data_table = self.concat_tables(existing_data, new_data)
        self.write_table(self.data_path_for(staging_root), data_table)

        episode_video_path = self.episode_video_path_for(staging_root, episode_index)
        self.write_combined_video(
            self.episode_images,
            path=episode_video_path,
        )

        episode_row = self.build_episode_metadata_table(
            episode_index=episode_index,
            start_index=start_index,
            length=length,
        )
        episodes_table = self.concat_tables(existing_episodes, episode_row)
        self.write_table(self.episodes_path_for(staging_root), episodes_table)
        self.write_tasks_table(path=self.tasks_path_for(staging_root))
        self.write_info(
            total_frames=self.total_frames + length,
            total_episodes=self.total_episodes + 1,
            path=self.info_path_for(staging_root),
        )
        self.write_stats(
            empty=False,
            data_path=self.data_path_for(staging_root),
            stats_path=self.stats_path_for(staging_root),
        )
        return [episode_video_path.relative_to(staging_root)]

    def commit_staged_dataset(
        self,
        staging_root: Path,
        backup_root: Path,
        extra_relative_paths: Optional[List[Path]] = None,
    ) -> None:
        for relative in self.managed_dataset_relative_paths(extra_relative_paths):
            source = staging_root / relative
            target = self.root / relative
            backup = backup_root / relative
            if target.exists():
                backup.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(target, backup)
            target.parent.mkdir(parents=True, exist_ok=True)
            source.replace(target)

        self.remove_tree(staging_root)

    def rollback_staged_save(
        self,
        staging_root: Path,
        backup_root: Path,
        extra_relative_paths: Optional[List[Path]] = None,
    ) -> None:
        for relative in self.managed_dataset_relative_paths(extra_relative_paths):
            target = self.root / relative
            backup = backup_root / relative
            if target.exists() and not backup.exists():
                target.unlink()
        if backup_root.exists():
            for backup_file in backup_root.rglob("*"):
                if not backup_file.is_file():
                    continue
                relative = backup_file.relative_to(backup_root)
                target = self.root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(backup_file, target)
        self.remove_tree(staging_root)
        self.remove_tree(backup_root)

    def managed_dataset_relative_paths(
        self, extra_relative_paths: Optional[List[Path]] = None
    ) -> List[Path]:
        paths = [
            Path("data/chunk-000/file-000.parquet"),
            Path("meta/info.json"),
            Path("meta/episodes/chunk-000/file-000.parquet"),
            Path("meta/stats.json"),
            Path("meta/tasks.parquet"),
        ]
        if extra_relative_paths:
            paths.extend(extra_relative_paths)
        return paths

    def remove_tree(self, path: Path) -> None:
        if path.exists():
            shutil.rmtree(path)

    def write_episode_bundle(
        self, episode_index: int, start_index: int, length: int
    ) -> Path:
        bundle_path = self.root / self.episode_bundle_dir / f"data{episode_index + 1}"
        if bundle_path.exists():
            raise RuntimeError(f"episode bundle already exists: {bundle_path}")

        data_path = bundle_path / "data" / "episode.parquet"
        video_path = bundle_path / "video" / "episode.mp4"
        frames_path = bundle_path / "frames"
        metadata_path = bundle_path / "metadata.json"

        episode_table = self.build_episode_table(
            episode_index=episode_index,
            start_index=start_index,
        )
        self.write_table(data_path, episode_table)
        self.write_combined_video(self.episode_images, path=video_path)

        frames_path.mkdir(parents=True, exist_ok=True)
        for frame_index, frame_rgb in enumerate(self.episode_images):
            frame_path = frames_path / f"frame_{frame_index:06d}.jpg"
            ok = cv2.imwrite(
                str(frame_path),
                np.ascontiguousarray(frame_rgb[:, :, ::-1]),
                [int(cv2.IMWRITE_JPEG_QUALITY), 90],
            )
            if not ok:
                raise RuntimeError(f"failed to write frame image: {frame_path}")

        metadata = {
            "episode_index": episode_index,
            "start_index": start_index,
            "length": length,
            "fps": self.fps,
            "task": self.task,
            "robot_type": self.robot_type,
            "data_path": "data/episode.parquet",
            "video_path": "video/episode.mp4",
            "frames_dir": "frames",
            "image_feature_key": self.image_feature_key,
            "observation_state_shape": [7],
            "action_shape": [7],
        }
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return bundle_path

    def build_episode_table(self, episode_index: int, start_index: int) -> pa.Table:
        states = [state.astype(np.float32).tolist() for state in self.episode_states]
        actions = [action.astype(np.float32).tolist() for action in self.episode_actions]
        joints = [state[:6].astype(np.float32).tolist() for state in self.episode_states]
        grippers = [[float(state[6])] for state in self.episode_states]
        timestamps = [float(i) / float(self.fps) for i in range(len(states))]
        frame_indices = list(range(len(states)))
        episode_indices = [episode_index] * len(states)
        global_indices = [start_index + i for i in range(len(states))]
        task_indices = [0] * len(states)
        video_rel = self.episode_video_path_for(self.root, episode_index).relative_to(
            self.root
        ).as_posix()
        video_refs = [
            {"path": video_rel, "timestamp": float(frame_index) / float(self.fps)}
            for frame_index in frame_indices
        ]

        schema = pa.schema(
            [
                ("observation.state", pa.list_(pa.float32(), list_size=7)),
                ("action", pa.list_(pa.float32(), list_size=7)),
                ("joints", pa.list_(pa.float32(), list_size=6)),
                ("gripper", pa.list_(pa.float32(), list_size=1)),
                ("actions", pa.list_(pa.float32(), list_size=7)),
                (
                    self.image_feature_key,
                    pa.struct(
                        [
                            ("path", pa.string()),
                            ("timestamp", pa.float32()),
                        ]
                    ),
                ),
                ("timestamp", pa.float32()),
                ("frame_index", pa.int64()),
                ("episode_index", pa.int64()),
                ("index", pa.int64()),
                ("task_index", pa.int64()),
            ]
        )
        return pa.Table.from_arrays(
            [
                pa.array(states, type=schema.field("observation.state").type),
                pa.array(actions, type=schema.field("action").type),
                pa.array(joints, type=schema.field("joints").type),
                pa.array(grippers, type=schema.field("gripper").type),
                pa.array(actions, type=schema.field("actions").type),
                pa.array(video_refs, type=schema.field(self.image_feature_key).type),
                pa.array(timestamps, type=pa.float32()),
                pa.array(frame_indices, type=pa.int64()),
                pa.array(episode_indices, type=pa.int64()),
                pa.array(global_indices, type=pa.int64()),
                pa.array(task_indices, type=pa.int64()),
            ],
            schema=schema,
        )

    def build_episode_metadata_table(
        self, episode_index: int, start_index: int, length: int
    ) -> pa.Table:
        schema = pa.schema(
            [
                ("episode_index", pa.int64()),
                ("tasks", pa.list_(pa.int64())),
                ("length", pa.int64()),
                ("dataset_from_index", pa.int64()),
                ("dataset_to_index", pa.int64()),
            ]
        )
        return pa.Table.from_arrays(
            [
                pa.array([episode_index], type=pa.int64()),
                pa.array([[0]], type=pa.list_(pa.int64())),
                pa.array([length], type=pa.int64()),
                pa.array([start_index], type=pa.int64()),
                pa.array([start_index + length], type=pa.int64()),
            ],
            schema=schema,
        )

    def write_tasks_table(self, path: Optional[Path] = None) -> None:
        output_path = path if path is not None else self.tasks_path
        table = pa.Table.from_pydict(
            {"task_index": [0], "task": [self.task]},
            schema=pa.schema([("task_index", pa.int64()), ("task", pa.string())]),
        )
        self.write_table(output_path, table)

    def write_info(
        self,
        total_frames: int,
        total_episodes: int,
        path: Optional[Path] = None,
    ) -> None:
        info = {
            "codebase_version": "v3.0",
            "repo_id": self.repo_id,
            "robot_type": self.robot_type,
            "total_episodes": int(total_episodes),
            "total_frames": int(total_frames),
            "total_tasks": 1,
            "chunks_size": 1000,
            "data_files_size_in_mb": 100,
            "video_files_size_in_mb": 200,
            "fps": int(self.fps),
            "splits": {"train": f"0:{int(total_episodes)}"},
            "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
            "video_path": (
                "videos/{video_key}/chunk-{chunk_index:03d}/"
                "episode_{episode_index:06d}.mp4"
            ),
            "features": self.features(),
        }
        output_path = path if path is not None else self.info_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(info, indent=4), encoding="utf-8")

    def write_stats(
        self,
        empty: bool,
        data_path: Optional[Path] = None,
        stats_path: Optional[Path] = None,
    ) -> None:
        input_data_path = data_path if data_path is not None else self.data_path
        output_stats_path = stats_path if stats_path is not None else self.stats_path
        stats = {}
        if not empty and input_data_path.exists():
            try:
                table = pq.read_table(input_data_path)
                for column_name in [
                    "observation.state",
                    "action",
                    "joints",
                    "gripper",
                    "actions",
                ]:
                    values = self.fixed_list_column_to_numpy(table[column_name])
                    stats[column_name] = {
                        "mean": values.mean(axis=0).astype(float).tolist(),
                        "std": values.std(axis=0).astype(float).tolist(),
                        "min": values.min(axis=0).astype(float).tolist(),
                        "max": values.max(axis=0).astype(float).tolist(),
                    }
            except Exception as exc:
                self.get_logger().warn(f"Could not compute dataset stats: {exc}")
        output_stats_path.parent.mkdir(parents=True, exist_ok=True)
        output_stats_path.write_text(json.dumps(stats, indent=4), encoding="utf-8")

    def fixed_list_column_to_numpy(self, column: pa.ChunkedArray) -> np.ndarray:
        return np.asarray(column.combine_chunks().to_pylist(), dtype=np.float32)

    def read_table_if_exists(self, path: Path) -> Optional[pa.Table]:
        if not path.exists():
            return None
        try:
            return pq.read_table(path)
        except Exception as exc:
            raise RuntimeError(f"Failed to read existing parquet {path}: {exc}") from exc

    def concat_tables(self, existing: Optional[pa.Table], new: pa.Table) -> pa.Table:
        if existing is None:
            return new
        if existing.num_rows == 0:
            return new
        try:
            return pa.concat_tables([existing, new], promote_options="default")
        except TypeError:
            return pa.concat_tables([existing, new], promote=True)

    def write_table(self, path: Path, table: pa.Table) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, path)

    def read_video_frames(self, path: Path) -> List[np.ndarray]:
        if not path.exists():
            return []
        capture = cv2.VideoCapture(str(path))
        frames: List[np.ndarray] = []
        while True:
            ok, frame_bgr = capture.read()
            if not ok:
                break
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            frames.append(np.ascontiguousarray(frame_rgb))
        capture.release()
        return frames

    def write_combined_video(
        self, frames: List[np.ndarray], path: Optional[Path] = None
    ) -> None:
        output_path = path if path is not None else self.video_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec[:4])
        writer = cv2.VideoWriter(
            str(output_path),
            fourcc,
            float(self.fps),
            (self.image_width, self.image_height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open MP4 writer at {output_path}")
        for frame_rgb in frames:
            writer.write(np.ascontiguousarray(frame_rgb[:, :, ::-1]))
        writer.release()

    def validate_dataset(
        self,
        root: Optional[Path] = None,
        total_frames: Optional[int] = None,
        total_episodes: Optional[int] = None,
        log_success: bool = True,
    ) -> None:
        dataset_root = root if root is not None else self.root
        expected_total_frames = (
            int(total_frames) if total_frames is not None else self.total_frames
        )
        expected_total_episodes = (
            int(total_episodes)
            if total_episodes is not None
            else self.total_episodes
        )
        data_path = self.data_path_for(dataset_root)
        tasks_path = self.tasks_path_for(dataset_root)
        episodes_path = self.episodes_path_for(dataset_root)
        stats_path = self.stats_path_for(dataset_root)

        errors = []
        if not data_path.exists():
            errors.append(f"missing data file: {data_path}")
        if not tasks_path.exists():
            errors.append(f"missing tasks file: {tasks_path}")
        if not episodes_path.exists():
            errors.append(f"missing episodes file: {episodes_path}")
        if not stats_path.exists():
            errors.append(f"missing stats file: {stats_path}")

        if errors:
            raise RuntimeError("; ".join(errors))

        data_table = pq.read_table(data_path)
        episodes_table = pq.read_table(episodes_path)

        if data_table.num_rows != expected_total_frames:
            errors.append(
                "data rows "
                f"({data_table.num_rows}) != total_frames ({expected_total_frames})"
            )
        if episodes_table.num_rows != expected_total_episodes:
            errors.append(
                "episode rows "
                f"({episodes_table.num_rows}) != total_episodes "
                f"({expected_total_episodes})"
            )

        unique_episodes = set(data_table["episode_index"].to_pylist())
        if len(unique_episodes) != expected_total_episodes:
            errors.append(
                "unique episode_index count "
                f"({len(unique_episodes)}) != total_episodes "
                f"({expected_total_episodes})"
            )
        if "length" in episodes_table.column_names:
            lengths_sum = sum(int(value) for value in episodes_table["length"].to_pylist())
            if lengths_sum != expected_total_frames:
                errors.append(
                    f"episode metadata lengths sum ({lengths_sum}) != "
                    f"total_frames ({expected_total_frames})"
                )
            self.validate_episode_videos(dataset_root, episodes_table, errors)

        self.validate_vector_shape(data_table, "observation.state", 7, errors)
        self.validate_vector_shape(data_table, "action", 7, errors)
        self.validate_episode_timestamps(data_table, errors)
        self.validate_stats_file(data_table, stats_path, errors)
        self.validate_info_paths(
            dataset_root,
            expected_total_frames,
            expected_total_episodes,
            errors,
        )

        if errors:
            raise RuntimeError("Dataset validation failed: " + "; ".join(errors))
        if log_success:
            self.get_logger().info(
                f"Validated dataset: {expected_total_episodes} episodes, "
                f"{expected_total_frames} frames, episode videos match parquet rows."
            )

    def validate_episode_videos(
        self, root: Path, episodes_table: pa.Table, errors: List[str]
    ) -> None:
        episode_indices = episodes_table["episode_index"].to_pylist()
        lengths = episodes_table["length"].to_pylist()
        total_video_frames = 0
        for episode_index_raw, length_raw in zip(episode_indices, lengths):
            episode_index = int(episode_index_raw)
            expected_length = int(length_raw)
            video_path = self.episode_video_path_for(root, episode_index)
            if not video_path.exists():
                fallback_video_path = self.episode_video_path_for(
                    self.root, episode_index
                )
                if root != self.root and fallback_video_path.exists():
                    video_path = fallback_video_path
                else:
                    errors.append(f"missing episode video file: {video_path}")
                    continue
            actual_frames = self.count_video_frames(video_path)
            total_video_frames += actual_frames
            if actual_frames != expected_length:
                errors.append(
                    f"episode {episode_index} video frames ({actual_frames}) != "
                    f"episode length ({expected_length})"
                )
        if total_video_frames and total_video_frames != sum(int(v) for v in lengths):
            errors.append(
                f"episode video frames total ({total_video_frames}) != "
                f"episode lengths total ({sum(int(v) for v in lengths)})"
            )

    def validate_vector_shape(
        self, table: pa.Table, column_name: str, expected: int, errors: List[str]
    ) -> None:
        if column_name not in table.column_names:
            errors.append(f"missing column {column_name}")
            return
        first = table[column_name][0].as_py() if table.num_rows else []
        if len(first) != expected:
            errors.append(f"{column_name} shape expected {expected}, got {len(first)}")

    def validate_episode_timestamps(self, table: pa.Table, errors: List[str]) -> None:
        if table.num_rows == 0:
            return
        timestamps = np.asarray(table["timestamp"].to_pylist(), dtype=np.float32)
        frame_indices = np.asarray(table["frame_index"].to_pylist(), dtype=np.int64)
        episode_indices = np.asarray(table["episode_index"].to_pylist(), dtype=np.int64)
        for episode_index in np.unique(episode_indices):
            mask = episode_indices == episode_index
            expected_frames = np.arange(mask.sum(), dtype=np.int64)
            expected_times = expected_frames.astype(np.float32) / float(self.fps)
            if not np.array_equal(frame_indices[mask], expected_frames):
                errors.append(f"frame_index is not continuous for episode {episode_index}")
            if not np.allclose(timestamps[mask], expected_times, atol=1e-4):
                errors.append(f"timestamp is not continuous for episode {episode_index}")

    def validate_stats_file(
        self, data_table: pa.Table, stats_path: Path, errors: List[str]
    ) -> None:
        try:
            stats = json.loads(stats_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            errors.append(f"invalid stats.json: {exc}")
            return
        for column_name in ["observation.state", "action"]:
            if column_name not in stats:
                errors.append(f"stats.json missing {column_name}")
                continue
            values = self.fixed_list_column_to_numpy(data_table[column_name])
            expected_mean = values.mean(axis=0)
            actual_mean = np.asarray(stats[column_name].get("mean", []), dtype=np.float32)
            if actual_mean.shape != expected_mean.shape:
                errors.append(f"stats.json {column_name} mean shape mismatch")
            elif not np.allclose(actual_mean, expected_mean, atol=1e-4):
                errors.append(f"stats.json {column_name} does not match all frames")

    def validate_info_paths(
        self,
        root: Path,
        expected_total_frames: int,
        expected_total_episodes: int,
        errors: List[str],
    ) -> None:
        try:
            info = json.loads(self.info_path_for(root).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            errors.append(f"invalid info.json: {exc}")
            return

        if int(info.get("total_frames", -1)) != expected_total_frames:
            errors.append(
                "info.json total_frames "
                f"({info.get('total_frames')}) != {expected_total_frames}"
            )
        if int(info.get("total_episodes", -1)) != expected_total_episodes:
            errors.append(
                "info.json total_episodes "
                f"({info.get('total_episodes')}) != {expected_total_episodes}"
            )

        data_path = info.get("data_path", "").format(chunk_index=0, file_index=0)
        for relative in [
            data_path,
            "meta/tasks.parquet",
            "meta/stats.json",
            "meta/episodes/chunk-000/file-000.parquet",
        ]:
            if relative and not (root / relative).exists():
                errors.append(f"info path missing: {relative}")

        video_template = info.get("video_path", "")
        for episode_index in range(expected_total_episodes):
            try:
                video_path = video_template.format(
                    video_key=self.image_feature_key,
                    chunk_index=0,
                    file_index=0,
                    episode_index=episode_index,
                )
            except (KeyError, IndexError, ValueError) as exc:
                errors.append(f"invalid info.json video_path template: {exc}")
                break
            if video_path and not (root / video_path).exists():
                fallback_path = self.root / video_path
                if root == self.root or not fallback_path.exists():
                    errors.append(f"info path missing: {video_path}")

    def count_video_frames(self, path: Path) -> int:
        capture = cv2.VideoCapture(str(path))
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if count <= 0:
            count = 0
            while True:
                ok, _ = capture.read()
                if not ok:
                    break
                count += 1
        capture.release()
        return count

    def clear_episode_buffers(self) -> None:
        self.episode_images.clear()
        self.episode_states.clear()
        self.episode_actions.clear()
        self.episode_timestamps.clear()

    def image_msg_to_rgb(self, msg: Image) -> np.ndarray:
        if msg.encoding.lower() in {"yuv422_yuy2", "yuyv", "yuyv422"}:
            return self.yuyv_to_rgb(msg)

        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        channels = channels_by_encoding.get(msg.encoding)
        if channels is None:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        array = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.width * channels
        if array.size < expected:
            raise ValueError("Image message data is shorter than expected.")
        array = array[:expected].reshape((msg.height, msg.width, channels))

        if msg.encoding == "rgb8":
            rgb = array
        elif msg.encoding == "bgr8":
            rgb = array[:, :, ::-1]
        elif msg.encoding == "rgba8":
            rgb = array[:, :, :3]
        elif msg.encoding == "bgra8":
            rgb = array[:, :, :3][:, :, ::-1]
        else:
            rgb = np.repeat(array, 3, axis=2)

        return np.ascontiguousarray(rgb)

    def yuyv_to_rgb(self, msg: Image) -> np.ndarray:
        bytes_per_row = msg.step if msg.step else msg.width * 2
        expected = bytes_per_row * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < expected:
            raise ValueError("YUYV image message data is shorter than expected.")

        rows = raw[:expected].reshape((msg.height, bytes_per_row))
        packed = rows[:, : msg.width * 2].reshape((msg.height, msg.width // 2, 4))

        y0 = packed[:, :, 0].astype(np.float32)
        u = packed[:, :, 1].astype(np.float32) - 128.0
        y1 = packed[:, :, 2].astype(np.float32)
        v = packed[:, :, 3].astype(np.float32) - 128.0

        rgb = np.empty((msg.height, msg.width, 3), dtype=np.uint8)
        rgb[:, 0::2, :] = self.yuv_to_rgb_pixels(y0, u, v)
        rgb[:, 1::2, :] = self.yuv_to_rgb_pixels(y1, u, v)
        return np.ascontiguousarray(rgb)

    def yuv_to_rgb_pixels(
        self, y: np.ndarray, u: np.ndarray, v: np.ndarray
    ) -> np.ndarray:
        r = y + 1.402 * v
        g = y - 0.344136 * u - 0.714136 * v
        b = y + 1.772 * u
        return np.stack(
            [
                np.clip(r, 0, 255),
                np.clip(g, 0, 255),
                np.clip(b, 0, 255),
            ],
            axis=-1,
        ).astype(np.uint8)

    def blank_image(self) -> np.ndarray:
        return np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

    def destroy_node(self) -> bool:
        if self.episode_states and self.save_partial_episode_on_shutdown:
            self.save_episode()
        elif self.episode_states:
            self.get_logger().info(
                "Discarding unfinished episode on shutdown. Call the save_episode "
                "service before shutdown to keep an episode."
            )
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LeRobotDemoRecorder()
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
