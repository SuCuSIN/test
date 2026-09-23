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
    launch_gripper = LaunchConfiguration("launch_gripper")
    gripper_force = LaunchConfiguration("gripper_force")
    gripper_hold_force = LaunchConfiguration("gripper_hold_force")
    gripper_min_width = LaunchConfiguration("gripper_min_width")
    gripper_max_width = LaunchConfiguration("gripper_max_width")
    gripper_send_rate = LaunchConfiguration("gripper_send_rate")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_gello_state_publisher"),
            "config",
            "ur3_gello.yaml",
        ]
    )

    gripper_node = Node(
        package="ur5e_gello_state_publisher",
        executable="rg6_tool_tcp_node",
        name="rg6_tool_tcp",
        output="screen",
        condition=IfCondition(launch_gripper),
        parameters=[
            {
                "transport": "tcp",
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "force": gripper_force,
                "hold_force": gripper_hold_force,
                "min_width": gripper_min_width,
                "max_width": gripper_max_width,
                "send_rate_hz": gripper_send_rate,
                "command_settle_sec": 0.15,
                "smoothing_alpha": 1.0,
                "command_deadband": 0.0025,
                "send_deadband": 0.004,
                "failed_send_cooldown_sec": 0.35,
                "status_read_rate_hz": 0.0,
                "respect_busy_status": False,
                "hold_on_grip_detected": False,
                "use_stall_grip_detection": False,
                "send_hold_on_grip": False,
                "send_stop_on_grip": False,
                "socket_timeout": 0.35,
                "inter_transaction_delay_sec": 0.12,
                "persistent_connection": False,
                "require_response": False,
                "reconnect_after_failures": 3,
                "reconnect_cooldown_sec": 1.5,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("gello_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.50.14"),
            DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
            DeclareLaunchArgument("launch_gripper", default_value="false"),
            DeclareLaunchArgument("gripper_force", default_value="20"),
            DeclareLaunchArgument("gripper_hold_force", default_value="10"),
            DeclareLaunchArgument("gripper_min_width", default_value="0.030"),
            DeclareLaunchArgument("gripper_max_width", default_value="0.13"),
            DeclareLaunchArgument("gripper_send_rate", default_value="3.0"),
            gripper_node,
            TimerAction(period=1.0, actions=[gello_node]),
        ]
    )
