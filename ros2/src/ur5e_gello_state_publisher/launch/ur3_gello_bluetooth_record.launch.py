from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    gello_port = LaunchConfiguration("gello_port")
    robot_ip = LaunchConfiguration("robot_ip")

    bluetooth_listen = LaunchConfiguration("bluetooth_listen")
    bluetooth_speed = LaunchConfiguration("bluetooth_speed")
    bluetooth_acceleration = LaunchConfiguration("bluetooth_acceleration")
    tactile_topic = LaunchConfiguration("tactile_topic")
    tactile_contact_threshold = LaunchConfiguration("tactile_contact_threshold")

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

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_gello_state_publisher"),
            "config",
            "ur3_gello.yaml",
        ]
    )

    bluetooth_node = Node(
        package="ur5e_gello_state_publisher",
        executable="bluetooth_gripper_bridge",
        name="bluetooth_gripper_bridge",
        output="screen",
        parameters=[
            {
                "port": bluetooth_listen,
                "topic": "/gello/gripper_raw",
                "controller_min_raw": 3459.0,
                "controller_max_raw": 4000.0,
                "controller_wrap_raw": 700.0,
                "output_min": 0,
                "output_max": 500,
                "publish_rate_hz": 30.0,
                "smoothing_alpha": 1.0,
                "max_step_per_update": 500.0,
                "command_deadband": 0,
                "speed": bluetooth_speed,
                "acceleration": bluetooth_acceleration,
                "tactile_topic": tactile_topic,
                "tactile_contact_threshold": tactile_contact_threshold,
            }
        ],
    )

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

    gello_node = Node(
        package="ur5e_gello_state_publisher",
        executable="ur5e_gello_publisher",
        name="ur5e_gello_publisher",
        output="screen",
        parameters=[
            config_file,
            {
                "port": gello_port,
                "robot_ip": robot_ip,
            },
        ],
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
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("gello_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.50.14"),
            DeclareLaunchArgument("bluetooth_listen", default_value="listen://127.0.0.1:15555"),
            DeclareLaunchArgument("bluetooth_speed", default_value="380"),
            DeclareLaunchArgument("bluetooth_acceleration", default_value="40"),
            DeclareLaunchArgument("tactile_topic", default_value=""),
            DeclareLaunchArgument("tactile_contact_threshold", default_value="25.0"),
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
            bluetooth_node,
            camera_node,
            TimerAction(period=1.0, actions=[gello_node]),
            TimerAction(period=3.0, actions=[recorder_node]),
        ]
    )
