import csv
import html
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from .timing_measurements import summarize, stats, timestamp_rows, TIMESTAMP_FIELDS
from .regrasp_report import episodes as regrasp_episodes, table as regrasp_table
from .regrasp_report import slip_intervals


DEFAULT_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JointSample = Tuple[float, List[float]]
ScalarSample = Tuple[float, float]
JOINT_LABELS = dict(zip(DEFAULT_JOINT_NAMES,
                       ("Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3")))


def joint_label(name):
    return JOINT_LABELS.get(name, name)


def latency_summary_rows(labels, intervals):
    rows = []
    for label in labels:
        values = [(lo + hi) / 2 for lo, hi in intervals.get(label, [])
                  if math.isfinite(lo) and math.isfinite(hi) and 0 <= lo <= hi]
        value = f"{statistics.mean(values):.1f} ms" if values else "Unavailable"
        rows.append(f'<tr><td>{html.escape(label)}</td><td title="{len(values)} qualifying events">{value}</td></tr>')
    return ''.join(rows)


def interpolate_vectors(
    samples: Sequence[JointSample], grid: Sequence[float]
) -> List[List[float]]:
    if len(samples) < 2:
        return []
    ordered = sorted(samples, key=lambda item: item[0])
    result: List[List[float]] = []
    index = 0
    for stamp in grid:
        while index + 1 < len(ordered) and ordered[index + 1][0] < stamp:
            index += 1
        if index + 1 >= len(ordered):
            break
        left_time, left_values = ordered[index]
        right_time, right_values = ordered[index + 1]
        if stamp < left_time:
            continue
        span = right_time - left_time
        ratio = 0.0 if span <= 0.0 else (stamp - left_time) / span
        result.append(
            [
                left + (right - left) * ratio
                for left, right in zip(left_values, right_values)
            ]
        )
    return result


def pearson(left: Sequence[float], right: Sequence[float]) -> Optional[float]:
    count = min(len(left), len(right))
    if count < 12:
        return None
    left = left[:count]
    right = right[:count]
    left_mean = sum(left) / count
    right_mean = sum(right) / count
    left_centered = [value - left_mean for value in left]
    right_centered = [value - right_mean for value in right]
    left_energy = sum(value * value for value in left_centered)
    right_energy = sum(value * value for value in right_centered)
    if left_energy <= 1e-12 or right_energy <= 1e-12:
        return None
    covariance = sum(a * b for a, b in zip(left_centered, right_centered))
    return covariance / math.sqrt(left_energy * right_energy)


def shifted_pairs(
    command: Sequence[float], actual: Sequence[float], lag_steps: int
) -> Tuple[Sequence[float], Sequence[float]]:
    count = min(len(command), len(actual))
    if lag_steps >= 0:
        return command[: count - lag_steps], actual[lag_steps:count]
    offset = -lag_steps
    return command[offset:count], actual[: count - offset]


def estimate_latency_steps(
    command_rows: Sequence[Sequence[float]],
    actual_rows: Sequence[Sequence[float]],
    dt: float,
    max_lag_sec: float,
    joint_index: Optional[int] = None,
) -> Tuple[Optional[int], Optional[float]]:
    count = min(len(command_rows), len(actual_rows))
    if count < 30 or dt <= 0.0:
        return None, None
    joint_indexes = (
        [joint_index]
        if joint_index is not None
        else list(range(min(len(command_rows[0]), len(actual_rows[0]))))
    )
    command_velocity: Dict[int, List[float]] = {}
    actual_velocity: Dict[int, List[float]] = {}
    active_indexes = []
    for index in joint_indexes:
        command = [row[index] for row in command_rows[:count]]
        actual = [row[index] for row in actual_rows[:count]]
        if max(command) - min(command) < math.radians(1.0):
            continue
        command_velocity[index] = [
            (command[i + 1] - command[i]) / dt for i in range(count - 1)
        ]
        actual_velocity[index] = [
            (actual[i + 1] - actual[i]) / dt for i in range(count - 1)
        ]
        active_indexes.append(index)
    if not active_indexes:
        return None, None

    max_steps = min(int(max_lag_sec / dt), max(1, (count - 2) // 3))
    best_steps: Optional[int] = None
    best_score: Optional[float] = None
    for lag_steps in range(-max_steps, max_steps + 1):
        scores = []
        for index in active_indexes:
            left, right = shifted_pairs(
                command_velocity[index], actual_velocity[index], lag_steps
            )
            score = pearson(left, right)
            if score is not None:
                scores.append(score)
        if not scores:
            continue
        score = sum(scores) / len(scores)
        if best_score is None or score > best_score + 1e-9 or (
            abs(score - best_score) <= 1e-9
            and best_steps is not None
            and abs(lag_steps) < abs(best_steps)
        ):
            best_steps = lag_steps
            best_score = score
    return best_steps, best_score


def angular_error(actual: float, command: float) -> float:
    return math.atan2(math.sin(actual - command), math.cos(actual - command))


def joint_metrics(
    command_rows: Sequence[Sequence[float]],
    actual_rows: Sequence[Sequence[float]],
    lag_steps: int,
    joint_index: int,
) -> Dict[str, float]:
    command = [row[joint_index] for row in command_rows]
    actual = [row[joint_index] for row in actual_rows]
    command, actual = shifted_pairs(command, actual, lag_steps)
    errors = [angular_error(measured, target) for target, measured in zip(command, actual)]
    if not errors:
        return {"rmse_deg": 0.0, "mae_deg": 0.0, "max_error_deg": 0.0}
    degrees = [math.degrees(value) for value in errors]
    return {
        "rmse_deg": math.sqrt(sum(value * value for value in degrees) / len(degrees)),
        "mae_deg": sum(abs(value) for value in degrees) / len(degrees),
        "max_error_deg": max(abs(value) for value in degrees),
    }


def svg_polyline(
    samples: Sequence[Tuple[float, float]],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    left: float,
    top: float,
    width: float,
    height: float,
) -> str:
    if not samples or x_max <= x_min or y_max <= y_min:
        return ""
    stride = max(1, math.ceil(len(samples) / 1600))
    points = []
    for stamp, value in samples[::stride]:
        x = left + (stamp - x_min) * width / (x_max - x_min)
        y = top + height - (value - y_min) * height / (y_max - y_min)
        points.append(f"{x:.1f},{y:.1f}")
    return " ".join(points)


def make_joint_svg(
    times: Sequence[float],
    command_rows: Sequence[Sequence[float]],
    actual_rows: Sequence[Sequence[float]],
    joint_names: Sequence[str],
    summary: Dict,
) -> str:
    width = 1400
    left = 145
    right = 35
    plot_width = width - left - right
    panel_height = 155
    panel_gap = 35
    joint_height = 2 * (panel_height + panel_gap) + 65
    top_margin = 95
    height = top_margin + len(joint_names) * joint_height + 20
    duration = max(0.001, times[-1] - times[0])
    origin = summary.get("plot_origin_pc_s")
    label_layout = {}
    joint_bands = {}
    for name in joint_names:
        max_lanes = 0
        for panel in (0, 1):
            lanes = []
            events = sorted((e for e in summary.get("plot_response_events", []) if e['joint'] == name),
                            key=lambda e: e['pc_observed_lower_ns'] if panel == 0 else e['command_send_start_ns'])
            for event in events:
                if origin is None:
                    continue
                start = (event['pc_observed_lower_ns'] if panel == 0 else event['command_send_start_ns']) / 1e9 - origin
                end = event['pc_observed_upper_ns'] / 1e9 - origin if panel == 0 else start
                if end < times[0] or start > times[-1]:
                    continue
                midpoint = (event['observation_lower_ms'] + event['observation_upper_ms']) / 2
                label = f"{midpoint:.1f} ms"
                label_width = len(label) * 7 + 8
                x = left + (max(times[0], start) - times[0]) / duration * plot_width
                label_x = max(left, min(left + plot_width - label_width, x - label_width / 2))
                lane = next((i for i, boundary in enumerate(lanes) if boundary + 6 <= label_x), len(lanes))
                if lane == len(lanes):
                    lanes.append(0)
                lanes[lane] = label_x + label_width
                label_layout[(name, panel, event['command_seq'])] = (label_x, lane, label)
            max_lanes = max(max_lanes, len(lanes))
        joint_bands[name] = max_lanes * 18
    height += 2 * sum(joint_bands.values())
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#172033}.title{font-size:28px;font-weight:700}.label{font-size:16px;font-weight:700}.tick{font-size:12px;fill:#526079}.metric{font-size:14px;fill:#334155}</style>',
        '<text x="32" y="40" class="title">GELLO command vs UR3 actual joint motion</text>',
        '<text x="32" y="68" class="metric">No time-shift alignment | Blue: GELLO command | Red: UR3 actual | Physical latency not measured</text>',
        '<text x="32" y="86" class="tick">Marker labels: observed latency interval midpoint (ms), not a measured exact latency or multi-event mean.</text>',
    ]
    for joint_index, joint_name in enumerate(joint_names):
        joint_top = top_margin + joint_index * joint_height + 2 * sum(joint_bands[n] for n in joint_names[:joint_index])
        command = [math.degrees(row[joint_index]) for row in command_rows]
        actual = [math.degrees(row[joint_index]) for row in actual_rows]
        y_min = min(command + actual)
        y_max = max(command + actual)
        padding = max(1.0, (y_max - y_min) * 0.10)
        y_min -= padding
        y_max += padding
        metric = summary["per_joint"][joint_name]
        parts.extend(
            [
                f'<text x="32" y="{joint_top + 18}" class="label">J{joint_index + 1} | {html.escape(joint_label(joint_name))}</text>',
                f'<text x="{left}" y="{joint_top + 40}" class="metric">RMSE {metric["rmse_deg"]:.2f} deg | MAE {metric["mae_deg"]:.2f} deg | Shared angle and time scales</text>',
            ]
        )
        for panel_index, (label, values, color) in enumerate((
            ("UR3 actual", actual, "#dc2626"),
            ("Controller", command, "#2563eb"),
        )):
            top = joint_top + 55 + joint_bands[joint_name] + panel_index * (panel_height + panel_gap + joint_bands[joint_name])
            parts.append(f'<g data-joint="{joint_index}" data-series="{label}">')
            parts.append(
                f'<rect x="{left}" y="{top}" width="{plot_width}" height="{panel_height}" fill="#f8fafc" stroke="#cbd5e1"/>'
            )
            parts.append(f'<text x="18" y="{top + 65}" class="label">{label}</text>')
            parts.append(f'<text x="18" y="{top + 85}" class="tick">angle (deg)</text>')
            for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
                y = top + panel_height * fraction
                value = y_max - (y_max - y_min) * fraction
                x = left + plot_width * fraction
                parts.extend([
                    f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_width}" y2="{y:.1f}" stroke="#e2e8f0"/>',
                    f'<text x="{left - 8}" y="{y + 4:.1f}" text-anchor="end" class="tick">{value:.1f}</text>',
                    f'<text x="{x:.1f}" y="{top + panel_height + 18}" text-anchor="middle" class="tick">{duration * fraction:.1f}s</text>',
                ])
            points = svg_polyline(
                list(zip(times, values)), times[0], times[-1],
                y_min, y_max, left, top, plot_width, panel_height,
            )
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
            origin = summary.get("plot_origin_pc_s")
            for event in summary.get("plot_response_events", []):
                if origin is None or event["joint"] != joint_name:
                    continue
                start = event["command_send_start_ns"] / 1e9 - origin
                end = start
                if panel_index == 0:
                    start = event["pc_observed_lower_ns"] / 1e9 - origin
                    end = event["pc_observed_upper_ns"] / 1e9 - origin
                if end < times[0] or start > times[-1]:
                    continue
                x1 = left + (max(times[0], start) - times[0]) / duration * plot_width
                x2 = left + (min(times[-1], end) - times[0]) / duration * plot_width
                tip = html.escape(
                    f"{joint_name} | command #{event['command_seq']} | "
                    f"send to observed response: {event['observation_lower_ms']:.2f} to "
                    f"{event['observation_upper_ms']:.2f} ms; label is interval midpoint, not a multi-event average or physical onset latency")
                parts.append(f'<g class="response-event" data-command="{event["command_seq"]}"><title>{tip}</title>')
                if panel_index == 0:
                    parts.append(f'<rect x="{x1:.2f}" y="{top}" width="{max(1, x2-x1):.2f}" height="{panel_height}" fill="#08916d" fill-opacity="0.08"/>')
                parts.append(f'<line x1="{x1:.2f}" x2="{x1:.2f}" y1="{top}" y2="{top+panel_height}" stroke="#087f5b" stroke-opacity="0.30" stroke-width="1" stroke-dasharray="4 3"/>')
                label_x, lane, midpoint_label = label_layout[(joint_name, panel_index, event['command_seq'])]
                parts.append(f'<text class="tick latency-midpoint" x="{label_x:.2f}" y="{top - 6 - lane * 18}">{midpoint_label}</text>')
                parts.append(f'<rect x="{max(left, x1-3):.2f}" y="{top}" width="{min(6, left+plot_width-max(left,x1-3)):.2f}" height="{panel_height}" fill="transparent"/>')
                parts.append('</g>')
            parts.append('</g>')
    parts.append("</svg>")
    return "\n".join(parts)


def make_gripper_svg(
    duration: float,
    input_samples: Sequence[ScalarSample],
    command_samples: Sequence[ScalarSample],
    actual_samples: Dict[int, Sequence[ScalarSample]],
    contact_samples: Sequence[Tuple[float, bool]],
    motion_events=(),
    command_frames=(),
    origin_pc_s=0.0,
    sensor_events=(),
    reinforcement_events=(),
) -> str:
    width = 1400
    frames = {(f['seq'], f['send_start_ns']): f for f in command_frames}
    labels = []
    lanes = []
    for event in motion_events:
        frame = frames.get((event['seq'], event['send_pc_ns']))
        start_ns = frame.get('received_ns') if frame else None
        basis = 'input' if start_ns is not None else 'send'
        start_ns = start_ns if start_ns is not None else event['send_pc_ns']
        lo, hi = event['observed_lower_pc_ns'], event['observed_upper_pc_ns']
        if not start_ns <= lo <= hi:
            continue
        start, end = start_ns / 1e9 - origin_pc_s, hi / 1e9 - origin_pc_s
        if not 0 <= start <= end <= duration:
            continue
        x = min(1095, 230 + start / max(duration, 0.001) * 1135)
        lane = next((i for i, edge in enumerate(lanes) if edge < x), len(lanes))
        if lane == len(lanes):
            lanes.append(0)
        lanes[lane] = x + 300
        labels.append((event, start, end, lane, basis,
                       (lo - start_ns) / 1e6, (hi - start_ns) / 1e6))
    label_space = max(24, len(lanes) * 18)
    contacts = [e for e in sensor_events if e.get('contact') and e.get('transition')
                and 0 <= e['observed_pc_ns'] / 1e9 - origin_pc_s <= duration]
    height = 600 + label_space + 24 * (len(contacts) + len(reinforcement_events))
    left = 230
    plot_width = width - left - 35
    panel_height = 125
    # Lever, command, actual: keep the command directly below measured motion.
    tops = [425 + label_space, 230 + label_space, 85 + label_space]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#172033}.title{font-size:28px;font-weight:700}.label{font-size:15px;font-weight:700}.tick{font-size:12px;fill:#526079}.note{font-size:14px;fill:#334155}</style>',
        '<text x="32" y="40" class="title">Gripper command, measured positions, and AnySkin contact</text>',
        '<text x="32" y="65" class="note">Actual servo samples are recorded whenever the Bluetooth controller reads ID 1 or ID 2.</text>',
    ]
    panels = [
        ("Controller lever (raw)", input_samples, "#2563eb"),
        ("Gripper command width (m)", command_samples, "#2563eb"),
    ]
    for panel_index, (label, samples, color) in enumerate(panels):
        top = tops[panel_index]
        parts.append(
            f'<rect x="{left}" y="{top}" width="{plot_width}" height="{panel_height}" fill="#f8fafc" stroke="#cbd5e1"/>'
        )
        parts.append(f'<text x="18" y="{top + 28}" class="label">{html.escape(label)}</text>')
        if samples:
            values = [value for _, value in samples]
            y_min = min(values)
            y_max = max(values)
            padding = max(0.001, (y_max - y_min) * 0.10)
            points = svg_polyline(
                samples, 0.0, duration, y_min - padding, y_max + padding,
                left, top, plot_width, panel_height,
            )
            parts.append(
                f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>'
            )
        else:
            parts.append(f'<text x="{left + 15}" y="{top + 35}" class="note">No samples</text>')

    top = tops[2]
    parts.append(
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{panel_height}" fill="#f8fafc" stroke="#cbd5e1"/>'
    )
    parts.append(f'<text x="18" y="{top + 28}" class="label">Actual gripper raw</text>')
    all_actual = [sample for samples in actual_samples.values() for sample in samples]
    if all_actual:
        values = [value for _, value in all_actual]
        y_min = min(values) - 10.0
        y_max = max(values) + 10.0
        for servo_id, color in ((1, "#059669"), (2, "#ea580c")):
            samples = actual_samples.get(servo_id, [])
            points = svg_polyline(
                samples, 0.0, duration, y_min, y_max,
                left, top, plot_width, panel_height,
            )
            parts.append(
                f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.5"/>'
            )
        parts.append(f'<text x="{left + 12}" y="{top + 22}" class="note">Green: ID 1 | Orange: ID 2</text>')
    else:
        parts.append(f'<text x="{left + 15}" y="{top + 35}" class="note">No actual position reads</text>')

    contact_on = False
    contact_start = 0.0
    for stamp, active in list(contact_samples) + [(duration, False)]:
        if active and not contact_on:
            contact_start = stamp
            contact_on = True
        elif not active and contact_on:
            x = left + contact_start * plot_width / max(duration, 0.001)
            end_x = left + stamp * plot_width / max(duration, 0.001)
            parts.append(
                f'<rect x="{x:.1f}" y="{top}" width="{max(1.0, end_x - x):.1f}" height="{panel_height}" fill="#ef4444" opacity="0.10"/>'
            )
            contact_on = False

    for interval in slip_intervals(sensor_events, origin_pc_s):
        start, end = max(0, interval['start_sec']), min(duration, interval['end_sec'])
        if end < start:
            continue
        x = left + start / max(duration, .001) * plot_width
        span = max(1, (end-start) / max(duration, .001) * plot_width)
        parts.append(f'<rect class="slip-interval" x="{x:.2f}" y="{tops[2]}" width="{span:.2f}" height="{panel_height}" fill="#9333ea" opacity="0.22"><title>Model slip interval {start:.3f}-{end:.3f}s; {interval["end_reason"]}; not ground-truth slip</title></rect>')
    parts.append(f'<text x="{left}" y="{tops[1]-8}" class="tick">Red shade: contact | Purple shade: model slip interval | Purple line: confirmed event | Blue line: reinforcement</text>')

    for event, start, end, lane, basis, lower, upper in labels:
        x1 = left + start / max(duration, 0.001) * plot_width
        x2 = left + end / max(duration, 0.001) * plot_width
        low_x = left + (event['observed_lower_pc_ns'] / 1e9 - origin_pc_s) / max(duration, 0.001) * plot_width
        parts.append(f'<rect x="{low_x:.2f}" y="{tops[2]}" width="{max(1, x2-low_x):.2f}" height="{panel_height}" fill="#059669" opacity="0.10"/>')
        marker_top = tops[1]
        parts.append(f'<line x1="{x1:.2f}" x2="{x1:.2f}" y1="{marker_top}" y2="{marker_top+panel_height}" stroke="#059669" opacity="0.3" stroke-dasharray="4 3"/>')
        text = f"ID{event['servo_id']} {basis}: {lower:.1f}-{upper:.1f} ms"
        parts.append(f'<text class="tick gripper-latency" x="{min(x1, width-305):.2f}" y="{80 + lane*18}"><title>Sequence {event["seq"]}; ROS input receipt or send to observed motion, including return transport and polling</title>{text}</text>')

    for index, event in enumerate(contacts):
        stamp = event['observed_pc_ns'] / 1e9 - origin_pc_s
        x = left + stamp / max(duration, 0.001) * plot_width
        values = []
        for sensor, row in enumerate(event['strengths']):
            finite = [v for v in row if math.isfinite(v)]
            values.append(f"S{sensor+1} max={max(finite):.1f}" if finite else f"S{sensor+1} unavailable")
        label = f"Contact {index+1} @ {stamp:.3f}s | " + ' | '.join(values)
        parts.append(f'<line x1="{x:.2f}" x2="{x:.2f}" y1="{tops[2]}" y2="{tops[1]+panel_height}" stroke="#dc2626" opacity="0.25" stroke-dasharray="4 3"/>')
        parts.append(f'<text class="tick sensor-contact" x="{left}" y="{585+label_space+index*24}">{html.escape(label)}</text>')

    for index, event in enumerate(reinforcement_events):
        name = 'Slip' if event['trigger'] == 'automatic_slip' else 'Manual probe'
        markers = [(event['time_sec'], '#9333ea', name)] + [
            (stamp, '#0284c7', 'Reinforcement command') for stamp in event['command_times_sec']]
        for stamp, color, title in markers:
            if not 0 <= stamp <= duration:
                continue
            x = left + stamp / max(duration, 0.001) * plot_width
            parts.append(f'<line x1="{x:.2f}" x2="{x:.2f}" y1="{tops[2]}" y2="{tops[1]+panel_height}" stroke="{color}" stroke-dasharray="3 3"><title>{title} event {index+1} @ {stamp:.3f}s</title></line>')
        movement = ', '.join(f"ID{i}: {event[f'id{i}_observed_closing_raw'] if event[f'id{i}_observed_closing_raw'] is not None else 'unavailable'} raw" for i in (1, 2))
        label = f"{name} {index+1} @ {event['time_sec']:.3f}s | {event['command_stages']} command stages | observed closing: {movement}"
        parts.append(f'<text class="tick" x="{left}" y="{585+label_space+(len(contacts)+index)*24}">{html.escape(label)}</text>')

    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = left + plot_width * fraction
        parts.append(
            f'<text x="{x:.1f}" y="{height - 15}" text-anchor="middle" class="tick">{duration * fraction:.1f}s</text>'
        )
    parts.append("</svg>")
    return "\n".join(parts)


class TrackingReportRecorder(Node):
    def __init__(self) -> None:
        super().__init__("gello_ur3_tracking_report")
        self.declare_parameter("controller_joint_topic", "/gello/joint_states")
        self.declare_parameter("robot_joint_topic", "/joint_states")
        self.declare_parameter("gripper_input_topic", "/gello/gripper_raw")
        self.declare_parameter(
            "gripper_command_topic", "/onrobot/finger_width_controller/commands"
        )
        self.declare_parameter(
            "gripper_actual_topic", "/gello/gripper_actual_raw"
        )
        self.declare_parameter(
            "gripper_contact_topic", "/gello/gripper_contact_hold"
        )
        self.declare_parameter("output_dir", "tracking_reports")
        self.declare_parameter("max_lag_sec", 0.75)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.max_lag_sec = max(0.05, float(self.get_parameter("max_lag_sec").value))
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.started_monotonic = time.monotonic()
        self.started_wall = datetime.now().astimezone()
        self.saved = False
        self.controller_samples: List[JointSample] = []
        self.robot_samples: List[JointSample] = []
        self.gripper_input_samples: List[ScalarSample] = []
        self.gripper_command_samples: List[ScalarSample] = []
        self.gripper_actual_samples: Dict[int, List[ScalarSample]] = {1: [], 2: []}
        self.gripper_contact_samples: List[Tuple[float, bool]] = []
        self.last_contact: Optional[bool] = None
        self.timing_frames = []
        self.gripper_timing_frames = []
        self.gripper_diagnostic_frames = []
        self.gripper_motion_frames = []
        self.gripper_sensor_frames = []
        self.create_subscription(String, "/gello/control_timing", self.timing_callback, 100)

        self.create_subscription(
            JointState,
            str(self.get_parameter("controller_joint_topic").value),
            self.controller_callback,
            100,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("robot_joint_topic").value),
            self.robot_callback,
            100,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("gripper_input_topic").value),
            self.gripper_input_callback,
            100,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("gripper_command_topic").value),
            self.gripper_command_callback,
            100,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("gripper_actual_topic").value),
            self.gripper_actual_callback,
            100,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("gripper_contact_topic").value),
            self.gripper_contact_callback,
            100,
        )
        self.get_logger().info(
            "Tracking report recording started. Ctrl+C will save HTML, SVG, CSV, and JSON."
        )

    def elapsed(self) -> float:
        return time.monotonic() - self.started_monotonic

    def timing_callback(self, message):
        try:
            frame = json.loads(message.data)
            if isinstance(frame, dict) and frame.get('component') in (
                    'gripper_motor_diagnostic', 'gripper_regrasp_position', 'gripper_regrasp_event',
                    'gripper_slip_observation'):
                self.gripper_diagnostic_frames.append(frame)
                return
            if isinstance(frame, dict) and "seq" in frame:
                if frame.get("component") == "gripper_sensor":
                    self.gripper_sensor_frames.append(frame)
                elif frame.get("component") == "gripper_motion":
                    self.gripper_motion_frames.append(frame)
                elif frame.get("component") == "gripper":
                    self.gripper_timing_frames.append(frame)
                else:
                    self.timing_frames.append(frame)
        except (ValueError, TypeError):
            self.get_logger().warn("Invalid source timing frame")

    def values_for_joints(self, message: JointState) -> Optional[List[float]]:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in self.joint_names):
            return None
        return [float(positions[name]) for name in self.joint_names]

    def controller_callback(self, message: JointState) -> None:
        values = self.values_for_joints(message)
        if values is not None:
            self.controller_samples.append((self.elapsed(), values))

    def robot_callback(self, message: JointState) -> None:
        values = self.values_for_joints(message)
        if values is not None:
            self.robot_samples.append((self.elapsed(), values))

    def gripper_input_callback(self, message: Float32) -> None:
        self.gripper_input_samples.append((self.elapsed(), float(message.data)))

    def gripper_command_callback(self, message: Float64MultiArray) -> None:
        if message.data:
            self.gripper_command_samples.append((self.elapsed(), float(message.data[0])))

    def gripper_actual_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) < 2:
            return
        servo_id = int(message.data[0])
        if servo_id in self.gripper_actual_samples:
            self.gripper_actual_samples[servo_id].append(
                (self.elapsed(), float(message.data[1]))
            )

    def gripper_contact_callback(self, message: Bool) -> None:
        active = bool(message.data)
        if active == self.last_contact:
            return
        self.last_contact = active
        self.gripper_contact_samples.append((self.elapsed(), active))

    def prepare_joint_data(self):
        controller_samples = self.controller_samples
        robot_samples = self.robot_samples
        if self.timing_frames:
            controller_samples = [(f["send_start_ns"] / 1e9, f["command_q"])
                                  for f in self.timing_frames if f.get("send_ok")]
            robot_samples = [(f["observed_ns"] / 1e9, f["robot_q"])
                             for f in self.timing_frames if f.get("robot_sample_consistent")]
        if len(controller_samples) < 3 or len(robot_samples) < 3:
            return None
        start = max(controller_samples[0][0], robot_samples[0][0])
        end = min(controller_samples[-1][0], robot_samples[-1][0])
        command_deltas = [
            right[0] - left[0]
            for left, right in zip(controller_samples, controller_samples[1:])
            if right[0] > left[0]
        ]
        if end <= start or not command_deltas:
            return None
        dt = max(0.008, min(0.05, statistics.median(command_deltas)))
        count = int((end - start) / dt) + 1
        if count < 3:
            return None
        grid = [start + index * dt for index in range(count)]
        command_rows = interpolate_vectors(controller_samples, grid)
        actual_rows = interpolate_vectors(robot_samples, grid)
        count = min(len(grid), len(command_rows), len(actual_rows))
        if count < 3:
            return None
        relative_times = [grid[index] - grid[0] for index in range(count)]
        self.plot_origin_pc_s = grid[0]
        return relative_times, command_rows[:count], actual_rows[:count], dt

    def save_report(self) -> Optional[Path]:
        if self.saved:
            return None
        self.saved = True
        finished = datetime.now().astimezone()
        duration = max(0.001, self.elapsed())
        session_name = self.started_wall.strftime("%Y%m%d_%H%M%S")
        report_dir = self.output_dir.resolve() / session_name
        report_dir.mkdir(parents=True, exist_ok=True)
        prepared = self.prepare_joint_data()
        summary = {
            "status": "ok" if prepared is not None else "insufficient_joint_samples",
            "started_at": self.started_wall.isoformat(),
            "finished_at": finished.isoformat(),
            "duration_sec": round(duration, 3),
            "controller_samples": len(self.controller_samples),
            "robot_samples": len(self.robot_samples),
            "estimated_latency_ms": None,
            "latency_correlation": None,
            "aggregate": {},
            "per_joint": {},
            "latency_note": (
                "Physical one-way latency is not measured. Software durations use source monotonic timestamps. "
                "Response intervals measure host-observed feedback threshold crossings, including return transport and polling. "
                "Joint errors use same-time samples with no lag correction."
            ),
        }

        if prepared is not None:
            summary["plot_origin_pc_s"] = self.plot_origin_pc_s
            summary["plot_response_events"] = summarize(self.timing_frames, self.joint_names)["response_events"]
            times, command_rows, actual_rows, dt = prepared
            metric_lag = 0
            all_errors = []
            for index, name in enumerate(self.joint_names):
                metrics = joint_metrics(command_rows, actual_rows, metric_lag, index)
                metrics["latency_ms"] = None
                metrics["latency_correlation"] = None
                metrics = {
                    key: round(value, 4) if isinstance(value, float) else value
                    for key, value in metrics.items()
                }
                summary["per_joint"][name] = metrics
                command, actual = shifted_pairs(
                    [row[index] for row in command_rows],
                    [row[index] for row in actual_rows],
                    metric_lag,
                )
                all_errors.extend(
                    math.degrees(angular_error(measured, target))
                    for target, measured in zip(command, actual)
                )
            if all_errors:
                summary["aggregate"] = {
                    "rmse_deg": round(
                        math.sqrt(
                            sum(value * value for value in all_errors)
                            / len(all_errors)
                        ),
                        4,
                    ),
                    "mae_deg": round(
                        sum(abs(value) for value in all_errors) / len(all_errors), 4
                    ),
                    "max_error_deg": round(max(abs(value) for value in all_errors), 4),
                }

            with (report_dir / "joint_tracking.csv").open(
                "w", newline="", encoding="utf-8"
            ) as file:
                writer = csv.writer(file)
                writer.writerow(
                    ["time_sec"]
                    + [f"controller_{name}_rad" for name in self.joint_names]
                    + [f"ur3_{name}_rad" for name in self.joint_names]
                    + [f"same_time_error_{name}_rad" for name in self.joint_names]
                )
                for stamp, command, actual in zip(times, command_rows, actual_rows):
                    errors = [
                        angular_error(measured, target)
                        for target, measured in zip(command, actual)
                    ]
                    writer.writerow([f"{stamp:.6f}"] + command + actual + errors)
            (report_dir / "joint_tracking.svg").write_text(
                make_joint_svg(
                    times, command_rows, actual_rows, self.joint_names, summary
                ),
                encoding="utf-8",
            )
        else:
            with (report_dir / "joint_tracking.csv").open(
                "w", newline="", encoding="utf-8"
            ) as file:
                csv.writer(file).writerow(
                    ["time_sec"]
                    + [f"controller_{name}_rad" for name in self.joint_names]
                    + [f"ur3_{name}_rad" for name in self.joint_names]
                    + [f"same_time_error_{name}_rad" for name in self.joint_names]
                )
            (report_dir / "joint_tracking.svg").write_text(
                """<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="240" viewBox="0 0 1200 240">
<rect width="1200" height="240" fill="#ffffff"/>
<text x="40" y="92" font-family="Arial, sans-serif" font-size="28" fill="#172033">Insufficient joint samples</text>
<text x="40" y="138" font-family="Arial, sans-serif" font-size="20" fill="#536078">The controller and UR3 joint-state topics must both publish for at least three samples.</text>
</svg>""",
                encoding="utf-8",
            )

        with (report_dir / "gripper_events.csv").open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            writer.writerow(["time_sec", "event", "servo_id", "value"])
            for stamp, value in self.gripper_input_samples:
                writer.writerow([f"{stamp:.6f}", "controller_raw", "", value])
            for stamp, value in self.gripper_command_samples:
                writer.writerow([f"{stamp:.6f}", "command_width_m", "", value])
            for servo_id, samples in self.gripper_actual_samples.items():
                for stamp, value in samples:
                    writer.writerow([f"{stamp:.6f}", "actual_raw", servo_id, value])
            for stamp, active in self.gripper_contact_samples:
                writer.writerow([f"{stamp:.6f}", "anyskin_contact", "", int(active)])

        reinforcement = regrasp_episodes(self.gripper_diagnostic_frames,
                                        self.gripper_timing_frames, self.started_monotonic)
        summary['gripper_reinforcement_episodes'] = reinforcement
        intervals = slip_intervals(self.gripper_sensor_frames, self.started_monotonic)
        summary['gripper_slip_intervals'] = intervals
        with (report_dir / 'gripper_slip_intervals.csv').open('w', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=['start_sec', 'end_sec', 'end_reason'])
            writer.writeheader()
            writer.writerows(intervals)
        with (report_dir / 'gripper_reinforcement.csv').open('w', newline='', encoding='utf-8') as file:
            fields = ['episode_id', 'trigger', 'time_sec', 'command_stages',
                      'id1_commanded_closing_raw', 'id1_observed_closing_raw',
                      'id2_commanded_closing_raw', 'id2_observed_closing_raw',
                      'id1_observation_time_sec', 'id2_observation_time_sec', 'observation']
            writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(reinforcement)
        (report_dir / "gripper_timeline.svg").write_text(
            make_gripper_svg(
                duration,
                self.gripper_input_samples,
                self.gripper_command_samples,
                self.gripper_actual_samples,
                self.gripper_contact_samples,
                self.gripper_motion_frames,
                self.gripper_timing_frames,
                self.started_monotonic,
                self.gripper_sensor_frames,
                reinforcement,
            ),
            encoding="utf-8",
        )
        measured = summarize(self.timing_frames, self.joint_names)
        summary["timestamp_definitions"] = {
            "host_clock": "time.monotonic_ns; source PC clock, not UTC",
            "robot_clock": "RTDE timestamp in seconds since robot startup; not synchronized to host",
            "controller_read": "Host read start/end interval, not the exact physical lever motion time",
            "pc_observed_ns": "Host SDK getter observation time, not network packet arrival time",
            "robot_threshold_interval": "Robot sample bracket for 0.1 degree displacement; not exact motion onset",
            "row_pairing": "Same control cycle only; robot sample may precede this row's command",
            "controller_input_deg": "Reader-returned controller joint angle before alignment and command smoothing",
            "missing_values": "Blank means not recorded; old logs cannot recover missing timestamps or angles",
        }
        with (report_dir / "joint_timestamps.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=TIMESTAMP_FIELDS)
            writer.writeheader()
            writer.writerows(timestamp_rows(self.timing_frames, self.joint_names))
        measured["software_durations"]["gripper_send_to_ACK"] = stats([
            (f["ack_ns"] - f["send_start_ns"]) / 1e6 for f in self.gripper_timing_frames])
        gripper_latency_rows = []
        motion_by_command = {}
        for event in self.gripper_motion_frames:
            key = (event['seq'], event['send_pc_ns'])
            if (event['servo_id'] in (1, 2)
                    and event['observed_upper_pc_ns'] >= event['send_pc_ns']
                    and event['observed_lower_pc_ns'] <= event['observed_upper_pc_ns']
                    and abs(event['actual_raw'] - event['baseline_raw']) >= event['threshold_raw']):
                motion_by_command.setdefault(key, {})[event['servo_id']] = event
        for frame in self.gripper_timing_frames:
            received = frame.get("received_ns")
            sent, acknowledged = frame["send_start_ns"], frame["ack_ns"]
            row = {
                "command_seq": frame.get("seq"), "ros_received_pc_ns": received,
                "send_pc_ns": sent, "ack_pc_ns": acknowledged,
                "ros_receive_to_send_ms": (sent - received) / 1e6 if received is not None and sent >= received else None,
                "send_to_ack_ms": (acknowledged - sent) / 1e6,
                "physical_motion_latency_ms": None,
            }
            matched = motion_by_command.get((frame.get('seq'), sent), {})
            for servo_id in (1, 2):
                event = matched.get(servo_id)
                row[f'id{servo_id}_observed_lower_ms'] = max(0, (event['observed_lower_pc_ns'] - sent) / 1e6) if event else None
                row[f'id{servo_id}_observed_upper_ms'] = (event['observed_upper_pc_ns'] - sent) / 1e6 if event else None
            gripper_latency_rows.append(row)
        measured["software_durations"]["gripper_ROS_receive_to_send"] = stats([
            row["ros_receive_to_send_ms"] for row in gripper_latency_rows
            if row["ros_receive_to_send_ms"] is not None])
        with (report_dir / "gripper_latency.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["command_seq", "ros_received_pc_ns", "send_pc_ns",
                "ack_pc_ns", "ros_receive_to_send_ms", "send_to_ack_ms", "physical_motion_latency_ms",
                "id1_observed_lower_ms", "id1_observed_upper_ms", "id2_observed_lower_ms", "id2_observed_upper_ms"])
            writer.writeheader()
            writer.writerows(gripper_latency_rows)
        measured["gripper_note"] = "ESP32 ACK round trip includes firmware handling; not motor travel or physical contact latency."
        summary["timing_measurements"] = measured
        summary["joint_time_basis"] = "source monotonic timestamps" if self.timing_frames else "ROS receipt times"
        with (report_dir / "source_timing.jsonl").open("w", encoding="utf-8") as file:
            for frame in self.timing_frames + self.gripper_timing_frames + self.gripper_motion_frames:
                file.write(json.dumps(frame) + "\n")
        with (report_dir / "gripper_sensor_events.jsonl").open("w", encoding="utf-8") as file:
            for frame in self.gripper_sensor_frames:
                file.write(json.dumps(frame) + "\n")
        with (report_dir / "gripper_motor_diagnostics.jsonl").open("w", encoding="utf-8") as file:
            for frame in self.gripper_diagnostic_frames:
                file.write(json.dumps(frame) + "\n")
        with (report_dir / "response_observations.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=[
                "joint", "command_seq", "observation_lower_ms", "observation_upper_ms", "threshold_deg",
                "controller_read_start_ns", "controller_read_end_ns", "command_send_start_ns",
                "robot_threshold_lower_s", "robot_threshold_upper_s",
                "pc_observed_lower_ns", "pc_observed_upper_ns",
                "input_read_end_to_send_ms",
            ])
            writer.writeheader()
            writer.writerows(measured["response_events"])
        def number(value):
            return "not measured" if value is None else f"{value:.2f}"
        with (report_dir / "gripper_motion_latency.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["component", "seq", "servo_id", "send_pc_ns",
                "observed_lower_pc_ns", "observed_upper_pc_ns", "observation_lower_ms",
                "observation_upper_ms", "threshold_raw", "baseline_raw", "actual_raw"])
            writer.writeheader()
            writer.writerows(self.gripper_motion_frames)
        gripper_motion_html = "".join(
            f"<tr><td>{e['servo_id']}</td><td>{e['seq']}</td>"
            f"<td>{number(e['observation_lower_ms'])} to {number(e['observation_upper_ms'])}</td>"
            f"<td>{e['baseline_raw']} to {e['actual_raw']}</td></tr>"
            for e in self.gripper_motion_frames[:100]) or '<tr><td colspan="4">No qualifying position response events recorded</td></tr>'
        summary['gripper_motion_events'] = self.gripper_motion_frames
        gripper_summary_rows = []
        for servo_id in (1, 2):
            events = [e for e in self.gripper_motion_frames if e['servo_id'] == servo_id]
            if events:
                lower = sum(e['observation_lower_ms'] for e in events) / len(events)
                upper = sum(e['observation_upper_ms'] for e in events) / len(events)
                value = f"{lower:.2f} to {upper:.2f} ms (mean observation bounds)"
            else:
                value = "Unavailable: no qualifying position feedback; not zero latency"
            gripper_summary_rows.append(
                f"<tr><td>ID {servo_id}</td><td>{len(events)}</td><td>{value}</td></tr>")
        gripper_latency_overview = (
            '<section><h2>Gripper latency summary</h2>'
            '<p>Command send to observed position change, separately per motor. '
            'These are feedback-based intervals, not exact physical onset times. '
            'Position queries are deferred during target tracking, so samples may be sparse '
            'and are not representative of every movement. ACK timing below measures command handling only.</p>'
            '<table><tr><th>Motor</th><th>Qualified events</th><th>Mean response interval</th></tr>'
            + ''.join(gripper_summary_rows) + '</table></section>')
        def gripper_response_cell(row):
            labels = []
            for servo_id in (1, 2):
                lower, upper = row[f'id{servo_id}_observed_lower_ms'], row[f'id{servo_id}_observed_upper_ms']
                if lower is None:
                    labels.append(f"ID {servo_id}: no matching qualified position event")
                else:
                    labels.append(f"ID {servo_id}: {(lower + upper) / 2:.2f} ms midpoint ({lower:.2f} to {upper:.2f} ms)")
            return '<br>'.join(labels)
        gripper_latency_html = "".join(
            f"<tr><td>{row['command_seq']}</td><td>{number(row['ros_receive_to_send_ms'])}</td>"
            f"<td>{number(row['send_to_ack_ms'])}</td><td>{gripper_response_cell(row)}</td></tr>"
            for row in gripper_latency_rows[:100]) or '<tr><td colspan="4">No gripper timing samples recorded</td></tr>'
        timing_rows = "".join(
            f"<tr><td>{html.escape(name)}</td><td>{values['count']}</td>"
            f"<td>{number(values['median_ms'])}</td><td>{number(values['p95_ms'])}</td>"
            f"<td>{number(values['max_ms'])}</td></tr>"
            for name, values in measured["software_durations"].items())
        response_rows = "".join(
            f"<tr><td>{html.escape(joint_label(event['joint']))}</td><td>{event['command_seq']}</td>"
            f"<td>{event['observation_lower_ms']:.1f} to {event['observation_upper_ms']:.1f}</td></tr>"
            for event in measured["response_events"])
        if not response_rows:
            response_rows = '<tr><td colspan="3">No qualifying rest-to-motion events recorded</td></tr>'
        def seconds(value, nanoseconds=False):
            if value is None:
                return "not recorded"
            return f"{value / 1e9 if nanoseconds else value:.6f}"

        timestamp_event_rows = "".join(
            f"<tr><td>{html.escape(joint_label(event['joint']))}</td><td>{event['command_seq']}</td>"
            f"<td>{seconds(event['controller_read_start_ns'], True)} to {seconds(event['controller_read_end_ns'], True)}</td>"
            f"<td>{seconds(event['command_send_start_ns'], True)}</td>"
            f"<td>{seconds(event['robot_threshold_lower_s'])} to {seconds(event['robot_threshold_upper_s'])}</td>"
            f"<td>{seconds(event['pc_observed_lower_ns'], True)} to {seconds(event['pc_observed_upper_ns'], True)}</td>"
            f"<td>{number(event['input_read_end_to_send_ms'])}</td>"
            f"<td>{event['observation_lower_ms']:.2f} to {event['observation_upper_ms']:.2f}</td></tr>"
            for event in measured['response_events'][:100]
        ) or '<tr><td colspan="8">No qualifying events; individual samples are available in joint_timestamps.csv.</td></tr>'
        clock_notes = " ".join(summary["timestamp_definitions"].values())
        (report_dir / "tracking_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )

        metric_row_parts = []
        for name, metrics in summary["per_joint"].items():
            latency_value = metrics["latency_ms"]
            latency_text = "n/a" if latency_value is None else f"{latency_value:.1f}"
            metric_row_parts.append(
                "<tr>"
                f"<td>{html.escape(joint_label(name))}</td>"
                f"<td>{metrics['rmse_deg']:.2f}</td>"
                f"<td>{metrics['mae_deg']:.2f}</td>"
                f"<td>{metrics['max_error_deg']:.2f}</td>"
                f"<td>{latency_text}</td>"
                "</tr>"
            )
        metric_rows = "".join(metric_row_parts)
        joint_intervals = {}
        for event in measured['response_events']:
            delay = event.get('input_read_end_to_send_ms')
            if delay is not None and delay >= 0:
                joint_intervals.setdefault(joint_label(event['joint']), []).append((
                    event['observation_lower_ms'] + delay,
                    event['observation_upper_ms'] + delay))
        joint_overview_rows = latency_summary_rows(
            [joint_label(n) for n in self.joint_names], joint_intervals)
        command_lookup = {(f['seq'], f['send_start_ns']): f for f in self.gripper_timing_frames}
        gripper_intervals = {}
        for event in self.gripper_motion_frames:
            frame = command_lookup.get((event['seq'], event['send_pc_ns']))
            if frame is None or frame.get('received_ns') is None:
                continue
            received = frame['received_ns']
            gripper_intervals.setdefault(f"ID {event['servo_id']}", []).append((
                (event['observed_lower_pc_ns'] - received) / 1e6,
                (event['observed_upper_pc_ns'] - received) / 1e6))
        gripper_overview_rows = latency_summary_rows(['ID 1', 'ID 2'], gripper_intervals)
        object_rows = []
        for event in self.gripper_sensor_frames:
            if not event.get('contact') or not event.get('transition'):
                continue
            sensor_values = []
            for index, row in enumerate(event['strengths']):
                finite = [v for v in row if math.isfinite(v)]
                sensor_values.append(f"S{index+1}: {max(finite):.1f}" if finite else f"S{index+1}: unavailable")
            stamp = event['observed_pc_ns'] / 1e9 - self.started_monotonic
            object_rows.append(f'<tr><td>{len(object_rows)+1}</td><td>{stamp:.3f} s</td>'
                               f'<td>{html.escape(" | ".join(sensor_values))}</td>'
                               '<td>Not measured</td></tr>')
        object_rows_html = ''.join(object_rows) or '<tr><td colspan="4">No confirmed contact recorded</td></tr>'
        report_html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>GELLO UR3 tracking report</title>
<style>body{{font-family:Arial,sans-serif;margin:28px;color:#172033;background:#f6f8fb}}main{{max-width:1440px;margin:auto}}section{{background:white;border:1px solid #d7deea;padding:20px;margin:18px 0}}h1{{font-size:30px}}.metrics{{display:flex;gap:28px;flex-wrap:wrap}}.metric strong{{display:block;font-size:26px}}table{{border-collapse:collapse;width:100%}}th,td{{padding:9px;border-bottom:1px solid #d7deea;text-align:right}}th:first-child,td:first-child{{text-align:left}}img{{width:100%;height:auto}}</style>
</head><body><main><h1>GELLO controller vs UR3 tracking report</h1>
<p>{html.escape(self.started_wall.isoformat())} | duration {duration:.1f}s | status {summary['status']}</p>
<section><h2>UR3 latency summary</h2><table><tr><th>Joint</th><th>Mean response latency</th></tr>{joint_overview_rows}</table><p>Controller read end to observed robot response. Mean of qualifying interval midpoints, not exact physical onset; includes feedback transport.</p></section>
<section><h2>Gripper latency summary</h2><table><tr><th>Motor</th><th>Mean response latency</th></tr>{gripper_overview_rows}</table><p>ROS receipt of lever-derived input to observed motor response, matched by command sequence and send timestamp. Mean of interval midpoints; not physical lever onset. Missing position feedback is not zero latency.</p></section>
<details><summary>Open detailed latency measurements and timestamps</summary>
<section><h2>Physical one-way latency: not measured</h2><p>{html.escape(summary['latency_note'])}</p></section>
{gripper_latency_overview}
<section><h2>Separate controller and robot timestamps</h2><p>{html.escape(clock_notes)}</p><p>Timestamp columns are seconds; latency columns are milliseconds. Host and robot columns have independent origins: do not subtract them. Latencies use only the PC clock. Observed response latency includes feedback transport and polling, and is not physical motor onset latency. First 100 qualifying events shown; CSV contains all events.</p><div style="overflow-x:auto"><table><tr><th>Joint</th><th>Command sequence</th><th>Controller read interval (PC clock)</th><th>Command send (PC clock)</th><th>Robot displacement crossing (robot clock)</th><th>Feedback observation interval (PC clock)</th><th>Input read end to send latency (ms)</th><th>Send to observed response latency (ms, interval)</th></tr>{timestamp_event_rows}</table></div><p><a href="joint_timestamps.csv">All joint samples and separate timestamps (CSV)</a> | <a href="response_observations.csv">All event intervals (CSV)</a></p></section>
<section><h2>Measured software durations (ms)</h2><p>{html.escape(measured['software_definition'])} {html.escape(measured['gripper_note'])}</p><table><tr><th>Interval</th><th>Samples</th><th>Median</th><th>P95</th><th>Maximum</th></tr>{timing_rows}</table></section>
<section><h2>Gripper communication latency</h2><p>Milliseconds on the PC clock. ROS receipt is not physical lever input time; repeated target steps can share a receipt timestamp. Position events are matched by command sequence AND send timestamp, separately for each servo. The midpoint summarizes the observation interval; it is not exact physical onset. No match means no qualifying event was recorded for this command, not zero delay or proof of no movement. Includes feedback transport and polling. ACK confirms firmware handling, not motor motion. First 100 commands shown.</p><table><tr><th>Command sequence</th><th>ROS receipt to send (ms)</th><th>Send to ACK (ms)</th><th>Position-feedback response latency (ms)</th></tr>{gripper_latency_html}</table><p><a href="gripper_latency.csv">All gripper communication timings</a></p></section>
<section><h2>Gripper position-feedback response latency</h2><p>Host-observed 3-raw displacement crossing after a command from observed rest. Includes Bluetooth return transport and polling; not exact physical onset. IDs are queried alternately, at most one query per 100 ms plus communication time. Missing, stale or unqualified events are not assigned a latency. First 100 events shown.</p><table><tr><th>Servo ID</th><th>Command sequence</th><th>Send to observed motion (ms interval)</th><th>Actual raw position</th></tr>{gripper_motion_html}</table><p><a href="gripper_motion_latency.csv">All position-feedback response events</a></p></section>
<section><h2>Observed response intervals (ms)</h2><p>{html.escape(measured['response_definition'])}</p><table><tr><th>Joint</th><th>Command sequence</th><th>Observation interval</th></tr>{response_rows}</table><p><a href="source_timing.jsonl">Source timestamps</a> | <a href="response_observations.csv">Response observations</a></p></section>
</details>
<section><h2>Joint tracking</h2><p>Top: UR3 actual position. Bottom: controller-derived command after alignment and filtering, not raw lever input. Shared plot axis uses host elapsed time (command send / SDK observation), not robot uptime. Green dashed markers below show command send events; green bands above show feedback observation intervals. Hover a marker for the command sequence and observed response latency. Independent original timestamps and controller input angles are in joint_timestamps.csv.</p><div class="joint-graph">{(report_dir / 'joint_tracking.svg').read_text(encoding='utf-8')}</div><style>.joint-graph svg{{width:100%;height:auto}}</style></section>
<details><summary>Open joint tracking error details</summary><section><h2>Same-time joint errors (no lag correction)</h2><table><thead><tr><th>Joint</th><th>RMSE (deg)</th><th>MAE (deg)</th><th>Max error (deg)</th><th>Physical latency</th></tr></thead><tbody>{metric_rows}</tbody></table></section></details>
<section><h2>Gripper timeline</h2><p>Top: measured gripper motor positions (raw). Directly below: commanded width (m). Bottom: controller lever (raw), for reference. Panels share time, not position units. Red markers identify confirmed contact; labels show each sensor's maximum strength at detection. Contact samples are recorded at up to 10 Hz while the control worker runs; blocking motor requests can create gaps. <a href="gripper_sensor_events.jsonl">Sensor values, baseline, grouped deltas and margin</a></p><img src="gripper_timeline.svg" alt="Gripper timeline"></section>
<section><h2>Object contact observations</h2><p>Contact numbers match the gripper graph. Sensor strength is not calibrated force or mass. Weight estimation has not been implemented.</p><table><tr><th>Contact</th><th>Detection time</th><th>Sensor strength at detection</th><th>Weight</th></tr>{object_rows_html}</table></section>
<details><summary>Open raw recording files</summary><section><h2>Raw files</h2><p><a href="joint_tracking.csv">joint_tracking.csv</a> | <a href="gripper_events.csv">gripper_events.csv</a> | <a href="tracking_summary.json">tracking_summary.json</a></p><p>{html.escape(summary['latency_note'])}</p></section></details>
{regrasp_table(reinforcement)}
</main></body></html>"""
        (report_dir / "tracking_report.html").write_text(
            report_html, encoding="utf-8"
        )
        (self.output_dir.resolve() / "latest_report.txt").write_text(
            str(report_dir / "tracking_report.html"), encoding="utf-8"
        )
        self.get_logger().info(f"Tracking report saved to {report_dir}")
        print(f"Tracking report saved: {report_dir / 'tracking_report.html'}")
        return report_dir


def main(args=None):
    rclpy.init(args=args)
    node = TrackingReportRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
