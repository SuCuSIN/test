from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from ur5e_gello_state_publisher.ur5e_gello_publisher import UR5eGelloPublisher


def make_node(raw=3000):
    node = SimpleNamespace(
        reader=SimpleNamespace(gripper_servo_id=7, fresh_raw_positions={7: raw},
                               read_joints_and_gripper=lambda: ([0]*6, .5)),
        control_mode='rtde_servoj', gripper_control_mode='position',
        gripper_publisher=Mock(), gripper_raw_publisher=Mock(),
        gripper_command_publisher=Mock(), gripper_command_alt_publisher=Mock(),
        publish_gripper_command=True, gripper_to_width=lambda value: value,
        filter_gripper_width=Mock(side_effect=lambda value: value),
        should_publish_gripper_command=lambda width: True,
        get_clock=lambda: SimpleNamespace(now=lambda: 123))
    node.publish_gripper_input = lambda value: UR5eGelloPublisher.publish_gripper_input(node, value)
    return node


def test_fresh_lever_published_before_rtde_read_or_send():
    node = make_node()

    def robot_read():
        node.gripper_command_publisher.publish.assert_called_once()
        node.gripper_command_alt_publisher.publish.assert_called_once()
        raise RuntimeError('robot read sentinel')

    node.update_robot_joints_from_rtde = robot_read
    with pytest.raises(RuntimeError, match='robot read sentinel'):
        UR5eGelloPublisher.publish_state(node)


def test_cached_lever_does_not_renew_command_heartbeat():
    node = make_node(None)
    node.publish_gripper_input(.5)
    node.gripper_command_publisher.publish.assert_not_called()
    node.gripper_command_alt_publisher.publish.assert_not_called()
    node.filter_gripper_width.assert_not_called()
    assert not hasattr(node, 'last_gripper_command_publish_time')


def test_bench_mode_never_publishes_gripper_commands():
    node = make_node()
    node.body_resistance_bench = True
    UR5eGelloPublisher.publish_state(node)
    node.gripper_command_publisher.publish.assert_not_called()
