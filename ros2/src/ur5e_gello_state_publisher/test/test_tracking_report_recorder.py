import math
import xml.etree.ElementTree as ET

from ur5e_gello_state_publisher.tracking_report_recorder import (
    estimate_latency_steps,
    joint_metrics,
    make_joint_svg,
    make_gripper_svg,
    joint_label,
    latency_summary_rows,
)


def test_latency_summary_averages_midpoints_without_inventing_missing_values():
    rows = latency_summary_rows(['Base', 'Shoulder'],
                                {'Base': [(10, 30), (30, 50), (-1, 3), (float('nan'), 9)]})
    assert '30.0 ms' in rows
    assert '2 qualifying events' in rows
    assert 'Unavailable' in rows
    assert rows.index('Base') < rows.index('Shoulder')


def test_display_joint_names_preserve_unknown_ids():
    assert joint_label('shoulder_pan_joint') == 'Base'
    assert joint_label('shoulder_lift_joint') == 'Shoulder'
    assert joint_label('wrist_3_joint') == 'Wrist 3'
    assert joint_label('custom_joint') == 'custom_joint'


def test_gripper_command_is_directly_below_actual():
    root = ET.fromstring(make_gripper_svg(2, [], [], {}, []))
    labels = {node.text: float(node.attrib['y']) for node in
              root.findall('{http://www.w3.org/2000/svg}text') if node.get('class') == 'label'}
    actual = labels['Actual gripper raw']
    command = labels['Gripper command width (m)']
    lever = labels['Controller lever (raw)']
    assert actual < command < lever
    assert command - actual == 145


def test_gripper_contact_annotation_records_each_sensor_once():
    events = [dict(observed_pc_ns=1000000000, contact=True, transition=True,
                   strengths=[[2, 8, 3], [4, 5, 12]])]
    svg = make_gripper_svg(2, [], [], {}, [], sensor_events=events)
    ET.fromstring(svg)
    assert 'S1 max=8.0 | S2 max=12.0' in svg
    assert svg.count('class="tick sensor-contact"') == 1


def test_gripper_latency_uses_matched_input_and_is_not_duplicated():
    event = dict(seq=1, servo_id=1, send_pc_ns=1020000000,
                 observed_lower_pc_ns=1100000000, observed_upper_pc_ns=1120000000)
    frame = dict(seq=1, send_start_ns=1020000000, received_ns=1000000000)
    svg = make_gripper_svg(2, [(0, 3900)], [], {1: [(1.1, 2000)]}, [],
                           [event], [frame], 0)
    ET.fromstring(svg)
    assert svg.count('class="tick gripper-latency"') == 1
    assert 'ID1 input: 100.0-120.0 ms' in svg
    svg = make_gripper_svg(2, [], [], {}, [], [event], [], 0)
    assert 'ID1 send: 80.0-100.0 ms' in svg
    assert 'gripper-latency' not in make_gripper_svg(2, [], [], {}, [])


def test_estimates_positive_robot_tracking_delay():
    dt = 0.01
    delay_steps = 7
    command = [
        [
            math.sin(2.0 * math.pi * 0.45 * index * dt),
            0.5 * math.sin(2.0 * math.pi * 0.73 * index * dt),
        ]
        for index in range(600)
    ]
    actual = [command[max(0, index - delay_steps)][:] for index in range(600)]

    measured_steps, correlation = estimate_latency_steps(
        command, actual, dt, max_lag_sec=0.25
    )

    assert measured_steps is not None
    assert abs(measured_steps - delay_steps) <= 1
    assert correlation is not None and correlation > 0.98


def test_tracking_error_is_small_after_latency_alignment():
    delay_steps = 5
    command = [[0.002 * index] for index in range(300)]
    actual = [command[max(0, index - delay_steps)][:] for index in range(300)]

    metrics = joint_metrics(command, actual, delay_steps, 0)

    assert metrics["rmse_deg"] < 0.01
    assert metrics["mae_deg"] < 0.01


def test_joint_panels_separate_actual_above_controller_with_shared_scales():
    svg = make_joint_svg(
        [0.0, 1.0], [[0.0], [1.0]], [[0.0], [1.0]], ["joint_1"],
        {"per_joint": {"joint_1": {"rmse_deg": 0.0, "mae_deg": 0.0}}},
    )
    ns = {"s": "http://www.w3.org/2000/svg"}
    root = ET.fromstring(svg)
    panels = root.findall("s:g", ns)
    assert [p.attrib["data-series"] for p in panels] == ["UR3 actual", "Controller"]
    rects = [p.find("s:rect", ns) for p in panels]
    offset = float(rects[1].attrib["y"]) - float(rects[0].attrib["y"])
    assert offset > float(rects[0].attrib["height"])
    paths = [p.findall("s:polyline", ns) for p in panels]
    assert all(len(p) == 1 for p in paths)
    points = [[tuple(map(float, pair.split(","))) for pair in p[0].attrib["points"].split()] for p in paths]
    for upper, lower in zip(*points):
        assert upper[0] == lower[0]
        assert abs(lower[1] - upper[1] - offset) < 0.01
    assert "lag " not in svg
