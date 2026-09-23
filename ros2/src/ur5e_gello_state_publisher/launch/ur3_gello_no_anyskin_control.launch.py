"""UR3 plus Bluetooth gripper; no AnySkin acquisition or lever haptics."""
from launch import LaunchDescription
from pathlib import Path
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / 'custom_gripper_test').is_dir())
    config = PathJoinSubstitution([FindPackageShare('ur5e_gello_state_publisher'),
                                  'config', 'ur3_gello.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('gello_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033706-if00'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.50.14'),
        DeclareLaunchArgument('bluetooth_address', default_value='tcp://127.0.0.1:15555'),
        DeclareLaunchArgument('close_speed', default_value='2919'),
        DeclareLaunchArgument('close_acc', default_value='22'),
        DeclareLaunchArgument('tracking_report', default_value='true'),
        DeclareLaunchArgument('tracking_report_dir', default_value=str(root / 'custom_gripper_test/tracking_reports')),
        DeclareLaunchArgument('position_report_hz', default_value='5.0'),
        LogInfo(msg='AnySkin and lever haptic feedback OFF. No tactile contact stop.'),
        Node(package='ur5e_gello_state_publisher', executable='tracking_report_recorder',
             name='gello_ur3_tracking_report', output='screen',
             condition=IfCondition(LaunchConfiguration('tracking_report')), parameters=[{
                 'output_dir': LaunchConfiguration('tracking_report_dir'),
                 'gripper_command_topic': '/gello/no_anyskin_command_width',
                 'gripper_actual_topic': '/gello/gripper_actual_raw',
                 'gripper_contact_topic': '/gello/no_anyskin_contact_unused',
             }]),
        Node(package='ur5e_gello_state_publisher', executable='bluetooth_gripper_bridge',
             name='bluetooth_gripper_no_anyskin', output='screen', parameters=[{
                 'port': LaunchConfiguration('bluetooth_address'),
                 'topic': '/gello/gripper_raw',
                 'command_topic': '', 'tactile_topic': '',
                 'controller_min_raw': 3425.0, 'controller_max_raw': 3924.0,
                 'controller_wrap_raw': -1.0,
                 'output_min': 0, 'output_max': 500,
                 'publish_rate_hz': 60.0, 'smoothing_alpha': 1.0,
                 # PAIR_MOVE spans 500 units for 480 raw ticks of servo travel.
                 'max_step_per_update': 8.0 * 500.0 / 480.0, 'command_deadband': 1,
                 'speed': ParameterValue(LaunchConfiguration('close_speed'), value_type=int),
                 'acceleration': ParameterValue(LaunchConfiguration('close_acc'), value_type=int),
                 'position_report_hz': ParameterValue(LaunchConfiguration('position_report_hz'), value_type=float),
             }]),
        TimerAction(period=1.0, actions=[Node(
            package='ur5e_gello_state_publisher', executable='ur5e_gello_publisher',
            name='ur5e_gello_publisher', output='screen', parameters=[config, {
                'port': LaunchConfiguration('gello_port'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'enable_gripper_haptic_feedback': False,
            }])]),
    ])
