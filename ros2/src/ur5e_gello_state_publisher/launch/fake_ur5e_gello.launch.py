from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_gello_state_publisher"),
            "config",
            "fake_ur5e_gello.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML config file for the fake UR5e GELLO publisher.",
            ),
            Node(
                package="ur5e_gello_state_publisher",
                executable="fake_ur5e_gello_publisher",
                name="fake_ur5e_gello_publisher",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
