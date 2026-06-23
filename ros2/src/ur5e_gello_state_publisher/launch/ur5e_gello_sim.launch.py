from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    port = LaunchConfiguration("port")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_gello_state_publisher"),
            "config",
            "ur5e_gello_sim.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML config file for real GELLO input with UR fake hardware.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
                description="Serial port for the STS3215 adapter.",
            ),
            Node(
                package="ur5e_gello_state_publisher",
                executable="ur5e_gello_publisher",
                name="ur5e_gello_publisher",
                output="screen",
                parameters=[config_file, {"port": port}],
            ),
        ]
    )
