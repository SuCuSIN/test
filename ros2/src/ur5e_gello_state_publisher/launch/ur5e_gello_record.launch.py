from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_device = LaunchConfiguration("camera_device")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    camera_pixel_format = LaunchConfiguration("camera_pixel_format")
    camera_topic = LaunchConfiguration("camera_topic")

    dataset_root = LaunchConfiguration("dataset_root")
    repo_id = LaunchConfiguration("repo_id")
    task = LaunchConfiguration("task")
    recorder_fps = LaunchConfiguration("recorder_fps")

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="base_rgb_camera",
        output="screen",
        parameters=[
            {
                "video_device": camera_device,
                "image_width": camera_width,
                "image_height": camera_height,
                "pixel_format": camera_pixel_format,
                "framerate": camera_fps,
            }
        ],
        remappings=[("/image_raw", camera_topic)],
    )

    recorder_node = Node(
        package="ur5e_gello_state_publisher",
        executable="lerobot_demo_recorder",
        name="lerobot_demo_recorder",
        output="screen",
        parameters=[
            {
                "repo_id": repo_id,
                "root": dataset_root,
                "fps": recorder_fps,
                "task": task,
                "camera_topic": camera_topic,
                "image_height": camera_height,
                "image_width": camera_width,
                "image_feature_key": "observation.images.main",
                "require_home_for_split": True,
                "home_settle_sec": 0.5,
                "split_cooldown_sec": 1.0,
                "auto_save_enabled": False,
                "save_episode_bundles": False,
                "episode_bundle_dir": "episode_bundles",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_width", default_value="1920"),
            DeclareLaunchArgument("camera_height", default_value="1080"),
            DeclareLaunchArgument("camera_fps", default_value="30.0"),
            DeclareLaunchArgument("camera_pixel_format", default_value="mjpeg2rgb"),
            DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("dataset_root", default_value="lerobot_demos/ur5e_gello_rg6"),
            DeclareLaunchArgument("repo_id", default_value="local/ur5e_gello_rg6"),
            DeclareLaunchArgument("task", default_value="pick and place the object"),
            DeclareLaunchArgument("recorder_fps", default_value="10"),
            camera_node,
            TimerAction(period=2.0, actions=[recorder_node]),
        ]
    )
