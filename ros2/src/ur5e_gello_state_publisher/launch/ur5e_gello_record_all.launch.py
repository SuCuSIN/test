from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    gello_port = LaunchConfiguration("gello_port")
    robot_ip = LaunchConfiguration("robot_ip")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")

    launch_camera = LaunchConfiguration("launch_camera")
    camera_device = LaunchConfiguration("camera_device")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    camera_pixel_format = LaunchConfiguration("camera_pixel_format")
    camera_topic = LaunchConfiguration("camera_topic")

    launch_gripper = LaunchConfiguration("launch_gripper")
    gripper_force = LaunchConfiguration("gripper_force")
    gripper_hold_force = LaunchConfiguration("gripper_hold_force")
    gripper_send_rate = LaunchConfiguration("gripper_send_rate")

    launch_recorder = LaunchConfiguration("launch_recorder")
    dataset_root = LaunchConfiguration("dataset_root")
    repo_id = LaunchConfiguration("repo_id")
    task = LaunchConfiguration("task")
    recorder_fps = LaunchConfiguration("recorder_fps")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_gello_state_publisher"),
            "config",
            "ur5e_gello.yaml",
        ]
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="base_rgb_camera",
        output="screen",
        condition=IfCondition(launch_camera),
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

    gripper_node = Node(
        package="ur5e_gello_state_publisher",
        executable="rg6_tool_tcp_node",
        name="rg6_tool_tcp",
        output="screen",
        condition=IfCondition(launch_gripper),
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "force": gripper_force,
                "hold_force": gripper_hold_force,
                "send_rate_hz": gripper_send_rate,
                "smoothing_alpha": 1.0,
                "max_close_step_per_send": 0.008,
                "max_open_step_per_send": 0.0,
                "close_latch_release_m": 0.040,
                "status_read_rate_hz": 5.0,
                "actual_width_register": 267,
                "hold_on_grip_detected": True,
                "use_stall_grip_detection": True,
                "grip_stall_time_sec": 0.18,
                "grip_stall_width_epsilon": 0.0015,
                "grip_close_request_margin": 0.002,
                "send_hold_on_grip": False,
                "send_stop_on_grip": False,
                "grip_release_margin": 0.018,
                "socket_timeout": 0.45,
                "persistent_connection": False,
            }
        ],
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
        condition=IfCondition(launch_recorder),
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
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("gello_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.119"),
            DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_width", default_value="1920"),
            DeclareLaunchArgument("camera_height", default_value="1080"),
            DeclareLaunchArgument("camera_fps", default_value="30.0"),
            DeclareLaunchArgument("camera_pixel_format", default_value="mjpeg2rgb"),
            DeclareLaunchArgument("camera_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("launch_gripper", default_value="true"),
            DeclareLaunchArgument("gripper_force", default_value="65"),
            DeclareLaunchArgument("gripper_hold_force", default_value="35"),
            DeclareLaunchArgument("gripper_send_rate", default_value="8.0"),
            DeclareLaunchArgument("launch_recorder", default_value="true"),
            DeclareLaunchArgument("dataset_root", default_value="lerobot_demos/ur5e_gello_rg6"),
            DeclareLaunchArgument("repo_id", default_value="local/ur5e_gello_rg6"),
            DeclareLaunchArgument("task", default_value="pick and place the object"),
            DeclareLaunchArgument("recorder_fps", default_value="10"),
            camera_node,
            gripper_node,
            TimerAction(period=1.0, actions=[gello_node]),
            TimerAction(period=3.0, actions=[recorder_node]),
        ]
    )
