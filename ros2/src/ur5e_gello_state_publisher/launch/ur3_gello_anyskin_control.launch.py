from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _safe_gripper_script() -> str:
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "custom_gripper_test" / "bluetooth_anyskin_gripper_shell.py"
        if candidate.exists():
            return str(candidate)
    raise RuntimeError("Could not locate custom_gripper_test/bluetooth_anyskin_gripper_shell.py")


def _safe_gripper_config() -> str:
    return str(Path(_safe_gripper_script()).with_name("gripper_motion_ranges.json"))


def _tracking_report_dir() -> str:
    return str(Path(_safe_gripper_script()).with_name("tracking_reports"))


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    gello_port = LaunchConfiguration("gello_port")
    robot_ip = LaunchConfiguration("robot_ip")
    anyskin_ports = LaunchConfiguration("anyskin_ports")
    bluetooth_host = LaunchConfiguration("bluetooth_host")
    bluetooth_port = LaunchConfiguration("bluetooth_port")
    close_speed = LaunchConfiguration("close_speed")
    close_acc = LaunchConfiguration("close_acc")
    safe_close_steps = LaunchConfiguration("safe_close_steps")
    safe_close_step_sec = LaunchConfiguration("safe_close_step_sec")
    contact_margin = LaunchConfiguration("contact_margin")
    web_port = LaunchConfiguration("web_port")
    feedback_topic = LaunchConfiguration("feedback_topic")
    haptic_release_delta = LaunchConfiguration("haptic_release_delta")
    haptic_hold_speed = LaunchConfiguration("haptic_hold_speed")
    haptic_hold_acc = LaunchConfiguration("haptic_hold_acc")
    haptic_hold_duration_sec = LaunchConfiguration("haptic_hold_duration_sec")
    haptic_feedback_timeout_sec = LaunchConfiguration(
        "haptic_feedback_timeout_sec"
    )
    tracking_report = LaunchConfiguration("tracking_report")
    tracking_report_dir = LaunchConfiguration("tracking_report_dir")
    tracking_max_lag_sec = LaunchConfiguration("tracking_max_lag_sec")

    default_config = PathJoinSubstitution(
        [FindPackageShare("ur5e_gello_state_publisher"), "config", "ur3_gello.yaml"]
    )

    safe_gripper = ExecuteProcess(
        cmd=[
            "python3",
            _safe_gripper_script(),
            "--gripper-host",
            bluetooth_host,
            "--firmware-hold",
            LaunchConfiguration("firmware_hold"),
            "--gripper-port",
            bluetooth_port,
            "--gripper-connect-timeout-sec",
            "12",
            "--config",
            _safe_gripper_config(),
            "--anyskin-ports",
            anyskin_ports,
            "--contact-threshold",
            "150",
            "--calibration-samples",
            "300",
            "--empty-close-baseline-percentile",
            LaunchConfiguration("empty_close_baseline_percentile"),
            "--speed",
            close_speed,
            "--acc",
            close_acc,
            "--safe-contact-confirm-sec",
            "0.04",
            "--safe-contact-confirm-samples",
            "3",
            "--safe-empty-baseline-margin",
            contact_margin,
            "--safe-contact-min-sensors",
            "1",
            "--slip-regrasp",
            LaunchConfiguration("slip_regrasp"),
            "--slip-regrasp-mode",
            LaunchConfiguration("slip_regrasp_mode"),
            "--regrasp-diagnostics",
            LaunchConfiguration("regrasp_diagnostics"),
            "--lever-release-delta",
            haptic_release_delta,
            "--record-anyskin-raw",
            LaunchConfiguration("record_anyskin_raw"),
            "--require-startup-calibration",
            "--safe-close-steps",
            safe_close_steps,
            "--safe-close-step-sec",
            safe_close_step_sec,
            "--safe-stop-backoff-raw",
            "0",
            "--ros-command-topic",
            "/onrobot/finger_width_controller/commands",
            "--ros-tracking-mode",
            "position",
            "--ros-feedback-topic",
            feedback_topic,
            "--ros-actual-position-topic",
            "/gello/gripper_actual_raw",
            "--web-port",
            web_port,
        ],
        output="screen",
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
                "gripper_control_mode": "continuous",
                "gripper_command_publish_rate_hz": 60.0,
                "gripper_command_publish_deadband_m": 0.0,
                "gripper_command_deadband_m": 0.0005,
                "gripper_smoothing_alpha": 1.0,
                "gripper_reversal_deadband_m": 0.001,
                "gripper_close_latch_release_m": 0.001,
                "gripper_open_confirm_cycles": 1,
                "enable_gripper_haptic_feedback": True,
                "body_resistance": ParameterValue(
                    LaunchConfiguration("body_resistance"), value_type=bool),
                "body_resistance_bench": ParameterValue(
                    LaunchConfiguration("body_resistance_bench"), value_type=bool),
                "gripper_haptic_feedback_topic": ParameterValue(
                    feedback_topic, value_type=str
                ),
                "gripper_haptic_release_delta": ParameterValue(
                    haptic_release_delta, value_type=float
                ),
                "gripper_haptic_hold_speed": ParameterValue(
                    haptic_hold_speed, value_type=int
                ),
                "gripper_haptic_hold_acceleration": ParameterValue(
                    haptic_hold_acc, value_type=int
                ),
                "gripper_haptic_hold_duration_sec": ParameterValue(
                    haptic_hold_duration_sec, value_type=float
                ),
                "gripper_haptic_feedback_timeout_sec": ParameterValue(
                    haptic_feedback_timeout_sec, value_type=float
                ),
            },
        ],
    )

    tracking_report_node = Node(
        package="ur5e_gello_state_publisher",
        executable="tracking_report_recorder",
        name="gello_ur3_tracking_report",
        output="screen",
        condition=IfCondition(tracking_report),
        parameters=[
            {
                "controller_joint_topic": "/gello/joint_states",
                "robot_joint_topic": "/joint_states",
                "gripper_input_topic": "/gello/gripper_raw",
                "gripper_command_topic": "/onrobot/finger_width_controller/commands",
                "gripper_actual_topic": "/gello/gripper_actual_raw",
                "gripper_contact_topic": feedback_topic,
                "output_dir": tracking_report_dir,
                "max_lag_sec": ParameterValue(
                    tracking_max_lag_sec, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("gello_port", default_value="/dev/ttyACM2"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.50.14"),
            DeclareLaunchArgument(
                "anyskin_ports", default_value="/dev/ttyACM0,/dev/ttyACM1"
            ),
            DeclareLaunchArgument("bluetooth_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("bluetooth_port", default_value="15555"),
            DeclareLaunchArgument("close_speed", default_value="2919"),
            DeclareLaunchArgument("body_resistance", default_value="false"),
            DeclareLaunchArgument("body_resistance_bench", default_value="false"),
            DeclareLaunchArgument("firmware_hold", default_value="false"),
            DeclareLaunchArgument("close_acc", default_value="22"),
            DeclareLaunchArgument("safe_close_steps", default_value="120"),
            DeclareLaunchArgument("safe_close_step_sec", default_value="0.008"),
            DeclareLaunchArgument("contact_margin", default_value="20"),
            DeclareLaunchArgument("empty_close_baseline_percentile", default_value="75"),
            DeclareLaunchArgument("slip_regrasp", default_value="false"),
            DeclareLaunchArgument("slip_regrasp_mode", default_value="heuristic"),
            DeclareLaunchArgument("regrasp_diagnostics", default_value="false"),
            DeclareLaunchArgument("record_anyskin_raw", default_value="true"),
            DeclareLaunchArgument("web_port", default_value="8765"),
            DeclareLaunchArgument(
                "feedback_topic", default_value="/gello/gripper_contact_hold"
            ),
            DeclareLaunchArgument("haptic_release_delta", default_value="0.03"),
            DeclareLaunchArgument("haptic_hold_speed", default_value="120"),
            DeclareLaunchArgument("haptic_hold_acc", default_value="20"),
            DeclareLaunchArgument(
                "haptic_hold_duration_sec", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "haptic_feedback_timeout_sec", default_value="1.0"
            ),
            DeclareLaunchArgument("tracking_report", default_value="true"),
            DeclareLaunchArgument(
                "tracking_report_dir", default_value=_tracking_report_dir()
            ),
            DeclareLaunchArgument("tracking_max_lag_sec", default_value="0.75"),
            tracking_report_node,
            safe_gripper,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=safe_gripper,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="AnySkin Bluetooth gripper safety process stopped"
                            )
                        )
                    ],
                )
            ),
            TimerAction(period=14.0, actions=[gello_node]),
        ]
    )
