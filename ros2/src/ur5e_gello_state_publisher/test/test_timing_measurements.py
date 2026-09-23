import copy
import csv
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy

from ur5e_gello_state_publisher.timing_measurements import summarize, timestamp_rows
from ur5e_gello_state_publisher.tracking_report_recorder import TrackingReportRecorder
from ur5e_gello_state_publisher.ur5e_gello_publisher import UR5eGelloPublisher


def frames():
    result = []
    for i in range(40):
        stamp = i * 20_000_000
        result.append({"seq": i, "send_ok": True,
                       "read_start_ns": stamp + 100_000, "read_end_ns": stamp + 500_000,
                       "send_start_ns": stamp + 1_000_000, "send_end_ns": stamp + 4_000_000,
                       "observed_ns": stamp, "robot_stamp": i * 0.02,
                       "robot_sample_consistent": True,
                       "command_q": [0.01 if i >= 20 else 0] * 6,
                       "robot_q": [0.002 if i >= 25 else 0] * 6})
    return result


def test_source_durations_and_observation_bracket():
    report = summarize(frames(), ['joint'])
    assert report['software_durations']['servoJ_call']['median_ms'] == 3
    assert report['software_durations']['read_end_to_servoJ_return']['median_ms'] == 3.5
    assert report['physical_one_way_latency_ms'] is None
    assert len(report['response_events']) == 1
    event = report['response_events'][0]
    assert event['observation_lower_ms'] == 79
    assert event['observation_upper_ms'] == 99
    assert event['input_read_end_to_send_ms'] == 0.5
    assert event['controller_read_start_ns'] == 400_100_000
    assert event['controller_read_end_ns'] == 400_500_000
    assert event['robot_threshold_lower_s'] == 0.48
    assert event['robot_threshold_upper_s'] == 0.5


def test_independent_clock_origins_are_preserved_not_subtracted():
    data = frames()
    for frame in data:
        frame['robot_stamp'] += 10000
    event = summarize(data, ['joint'])['response_events'][0]
    assert event['robot_threshold_lower_s'] == 10000.48
    assert event['robot_threshold_upper_s'] == 10000.5
    assert event['observation_upper_ms'] == 99
    row = next(timestamp_rows(data, ['joint']))
    assert row['robot_sample_time_s'] == 10000
    assert row['controller_read_start_ns'] == 100_000
    assert row['controller_input_deg'] is None


def test_timestamp_rows_preserve_input_separately_from_command():
    data = frames()[:1]
    data[0]['controller_q'] = [1.0]
    data[0]['controller_target_q'] = [2.0]
    row = next(timestamp_rows(data, ['joint']))
    assert abs(row['controller_input_deg'] - 57.2957795) < .00001
    assert row['controller_aligned_target_deg'] > row['controller_input_deg']
    assert row['command_deg'] == 0


def test_servoj_timestamps_bracket_call_not_wait_period():
    node = object.__new__(UR5eGelloPublisher)
    node.rtde_control_interface = Mock()
    node.rtde_control_interface.servoJ.return_value = True
    node.rtde_velocity = node.rtde_acceleration = 0.5
    node.rtde_dt, node.rtde_lookahead_time, node.rtde_gain = 0.008, 0.08, 300
    with patch('ur5e_gello_state_publisher.ur5e_gello_publisher.time.monotonic_ns',
               side_effect=[100, 900]) as clock:
        node.send_rtde_servoj([0] * 6)
    assert node.last_servoj_timing == {'send_start_ns': 100, 'send_end_ns': 900, 'send_ok': True}
    assert clock.call_count == 2
    node.rtde_control_interface.waitPeriod.assert_called_once()


def test_inconsistent_rtde_snapshot_is_marked_not_usable_for_onset():
    node = object.__new__(UR5eGelloPublisher)
    node.rtde_receive_interface = SimpleNamespace(getTimestamp=Mock(side_effect=[1.0, 1.008]),
                                                  getActualQ=Mock(return_value=[0.1] * 6))
    node.update_robot_joints_from_rtde()
    assert node.last_robot_observation['robot_sample_consistent'] is False
    assert node.latest_robot_joints == [0.1] * 6


def test_no_data_means_not_measured():
    report = summarize([], ['joint'])
    assert report['status'] == 'not_recorded'
    assert report['software_durations']['servoJ_call']['median_ms'] is None
    assert report['response_events'] == []


def test_absent_motion_does_not_create_response():
    data = frames()
    for frame in data:
        frame['robot_q'] = [0] * 6
    assert summarize(data, ['joint'])['response_events'] == []


def test_failed_commands_are_excluded():
    data = frames()
    for frame in data:
        frame['send_ok'] = False
    report = summarize(data, ['joint'])
    assert report['response_events'] == []
    assert report['software_durations']['servoJ_call']['count'] == 0


def test_duplicate_robot_timestamp_cannot_prove_response():
    data = frames()
    for frame in data[21:]:
        frame['robot_stamp'] = data[20]['robot_stamp']
    assert summarize(data, ['joint'])['response_events'] == []


def test_feedback_gap_excludes_event():
    data = frames()
    data = data[:21] + data[28:]
    assert summarize(data, ['joint'])['response_events'] == []


def test_report_artifacts_contain_measurements_without_estimated_latency():
    rclpy.init()
    node = TrackingReportRecorder()
    try:
        with tempfile.TemporaryDirectory() as directory:
            node.output_dir = Path(directory)
            node.timing_frames = copy.deepcopy(frames())
            node.gripper_timing_frames = [{"component": "gripper", "seq": 0,
                "received_ns": 1000000, "send_start_ns": 3000000, "ack_ns": 12000000}]
            node.timing_callback(SimpleNamespace(data=json.dumps({
                'component': 'gripper_motion', 'seq': 0, 'servo_id': 1,
                'send_pc_ns': 3000000, 'observed_lower_pc_ns': 10000000,
                'observed_upper_pc_ns': 20000000, 'observation_lower_ms': 7.0,
                'observation_upper_ms': 17.0, 'threshold_raw': 3,
                'baseline_raw': 2000, 'actual_raw': 1996})))
            path = node.save_report()
            result = json.loads((path / 'tracking_summary.json').read_text())
            assert result['estimated_latency_ms'] is None
            with (path / 'gripper_latency.csv').open() as file:
                gripper_rows = list(csv.DictReader(file))
            assert gripper_rows[0]['ros_receive_to_send_ms'] == '2.0'
            assert gripper_rows[0]['send_to_ack_ms'] == '9.0'
            assert gripper_rows[0]['physical_motion_latency_ms'] == ''
            assert gripper_rows[0]['id1_observed_lower_ms'] == '7.0'
            assert gripper_rows[0]['id1_observed_upper_ms'] == '17.0'
            assert gripper_rows[0]['id2_observed_lower_ms'] == ''
            assert 'ID 1: 12.00 ms midpoint (7.00 to 17.00 ms)' in (path / 'tracking_report.html').read_text()
            with (path / 'gripper_motion_latency.csv').open() as file:
                motion_rows = list(csv.DictReader(file))
            assert motion_rows[0]['observation_upper_ms'] == '17.0'
            assert 'Gripper position-feedback response latency' in (path / 'tracking_report.html').read_text()
            assert len(result['timing_measurements']['response_events']) == 6
            assert (path / 'source_timing.jsonl').exists()
            with (path / 'joint_timestamps.csv').open() as file:
                rows = list(csv.DictReader(file))
            assert len(rows) == 240
            assert rows[0]['controller_read_start_ns'] == '100000'
            assert rows[0]['controller_input_deg'] == ''
            with (path / 'response_observations.csv').open() as file:
                events = list(csv.DictReader(file))
            assert len(events) == 6
            assert events[0]['robot_threshold_upper_s'] == '0.5'
            assert events[0]['input_read_end_to_send_ms'] == '0.5'
            report_html = (path / 'tracking_report.html').read_text()
            assert 'Input read end to send latency (ms)' in report_html
            assert 'Send to observed response latency (ms, interval)' in report_html
            assert '<td>0.50</td><td>79.00 to 99.00</td>' in report_html
            assert 'Separate controller and robot timestamps' in (path / 'tracking_report.html').read_text()
            svg = (path / 'joint_tracking.svg').read_text()
            assert svg.count('class="response-event"') == 12
            assert svg.count('class="tick latency-midpoint"') == 12
            assert '>89.0 ms</text>' in svg
            assert 'stroke-opacity="0.30"' in svg
            assert 'fill-opacity="0.08"' in svg
            assert 'command #20' in svg
            assert '79.00 to 99.00 ms' in svg
            assert '<div class="joint-graph"><svg' in report_html
            assert 'Physical one-way latency: not measured' in (path / 'tracking_report.html').read_text()
    finally:
        node.destroy_node()
        rclpy.shutdown()
