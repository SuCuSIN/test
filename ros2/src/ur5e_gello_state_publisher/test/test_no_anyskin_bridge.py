import socket
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from ur5e_gello_state_publisher.bluetooth_gripper_bridge import BluetoothGripperBridge as Bridge


def test_raw_input_to_tcp_command_without_calibration():
    sender, peer = socket.socketpair()
    with sender, peer:
        sender.setblocking(False)
        peer.settimeout(1)
        state = SimpleNamespace(
            minimum_raw=3425., maximum_raw=3924., wrap_raw=-1., encoder_ticks=4096.,
            output_min=0, output_max=500, latest_target=None, filtered_target=None,
            last_command=None, deadband=1, speed=2919, acceleration=22,
            alpha=1., max_step=8., socket=sender, serial_port=None,
            get_logger=lambda: Mock(), is_connected=lambda: True,
            apply_tactile_safety=lambda target: target,
            response_buffer='', poll_position_report=lambda: None,
            command_report_publisher=Mock(), input_min_width=.012, input_max_width=.13,
        )
        state.read_available = lambda: Bridge.read_available(state)
        state.write_command = lambda payload: Bridge.write_command(state, payload)
        Bridge.raw_callback(state, SimpleNamespace(data=3623.))
        Bridge.update(state)
        assert peer.recv(128) == b'PAIR_MOVE 302 2919 22\n'


def test_tcp_eof_is_not_treated_as_no_response():
    sender, peer = socket.socketpair()
    peer.close()
    with sender:
        with pytest.raises(ConnectionError, match='peer closed'):
            Bridge.read_available(SimpleNamespace(socket=sender))


def test_actual_report_requires_valid_position_response():
    state = SimpleNamespace(actual_publisher=Mock(), position_read_pending=2)
    Bridge.record_response(state, 'PAIR_MOVE sent: position=300')
    Bridge.record_response(state, 'POSITION id=2 logical=153 raw=2200 status=0x4')
    state.actual_publisher.publish.assert_not_called()
    Bridge.record_response(state, 'POSITION id=2 logical=153 raw=2200 status=0x0')
    assert list(state.actual_publisher.publish.call_args.args[0].data) == [2., 2200.]
    assert state.position_read_pending is None


def test_fragmented_position_reply_is_recorded_once():
    state = SimpleNamespace(actual_publisher=Mock(), position_read_pending=1,
                            latest_target=None, response_buffer='',
                            get_logger=lambda: Mock(), is_connected=lambda: True,
                            poll_position_report=lambda: None,
                            read_available=Mock(side_effect=['POS', 'ITION id=1 logical=-247 raw=1800 status=0x0\r\n']))
    state.record_response = lambda line: Bridge.record_response(state, line)
    Bridge.update(state)
    state.actual_publisher.publish.assert_not_called()
    Bridge.update(state)
    state.actual_publisher.publish.assert_called_once()
