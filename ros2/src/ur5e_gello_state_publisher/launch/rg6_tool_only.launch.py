from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    tcp_port = LaunchConfiguration("tcp_port")
    device_name = LaunchConfiguration("device_name")
    onrobot_type = LaunchConfiguration("onrobot_type")
    ns = LaunchConfiguration("ns")

    tool_communication = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tcp_port,
                "device_name": device_name,
            }
        ],
    )

    onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("onrobot_driver"),
                    "launch",
                    "onrobot_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "onrobot_type": onrobot_type,
            "connection_type": "serial",
            "device": device_name,
            "ns": ns,
            "launch_rviz": "false",
            "launch_rsp": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.119"),
            DeclareLaunchArgument("tcp_port", default_value="54321"),
            DeclareLaunchArgument("device_name", default_value="/tmp/ttyUR"),
            DeclareLaunchArgument("onrobot_type", default_value="rg6"),
            DeclareLaunchArgument("ns", default_value="onrobot"),
            tool_communication,
            TimerAction(period=2.0, actions=[onrobot_launch]),
        ]
    )
