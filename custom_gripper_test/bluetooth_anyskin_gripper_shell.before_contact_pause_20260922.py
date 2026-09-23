"""Bluetooth gripper shell with AnySkin contact stop.

Run a Windows COM-to-TCP bridge for the ESP32 Bluetooth gripper first, then run
this script in WSL so it can read AnySkin USB sensors and command the gripper.
"""

from __future__ import annotations

import argparse
import json
from http.server import ThreadingHTTPServer
import math
from pathlib import Path
import queue
import re
import socket
import threading
import time
from typing import Callable

from anyskin_web_viz import AnySkinWebHandler
from anyskin_raw_recorder import AnySkinRawRecorder
from gripper_position_tracking import PositionTracker
from gripper_regrasp import RegraspGuard, bounded_target
from gripper_motion_timing import GripperMotionTiming
from manual_gripper_shell import (
    AnySkinMonitor,
    backoff_toward_open,
    close_ratio_from_raws,
    empty_baseline_magnet_strengths,
    empty_baseline_strengths,
    max_finite,
    load_config,
    parse_float_list,
    parse_one_based_index_set,
    parse_ports,
    save_config,
    sensor_max_strengths,
)


DEFAULT_OPEN_RAWS = (2000, 2000)
DEFAULT_CLOSE_RAWS = (1520, 2480)
SERVO_MIDDLE_RAW = 2047
DEFAULT_SPEED = 120
DEFAULT_ACC = 8
DEFAULT_CONFIG = Path(__file__).with_name("gripper_motion_ranges.json")
POSITION_RE = re.compile(r"POSITION id=(?P<id>\d+).* raw=(?P<raw>-?\d+)")
COMMAND_ACK_TIMEOUT_SEC = 1.5
UNKNOWN_MATERIAL_HINT = {
    "label": "unknown",
    "peak_delta": None,
    "average_delta": None,
    "rise_rate": None,
    "contact_ratio": None,
    "elapsed_sec": None,
    "classification_basis": None,
    "note": "Run Close Safe to estimate object stiffness.",
}


def raw_to_logical(raw: int) -> int:
    return raw - SERVO_MIDDLE_RAW


class BluetoothGripperClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None
        self.socket_lock = threading.Lock()
        self.lines: queue.Queue[str] = queue.Queue()
        self.stop_event = threading.Event()
        self.reader: threading.Thread | None = None
        self.last_raws: tuple[int, int] | None = None
        self.position_callback: Callable[[int, int], None] | None = None
        self.trace_commands = False
        self.last_command = None
        self.last_received_line = None

    def connect(self) -> None:
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(None)
        with self.socket_lock:
            self.stop_event.clear()
            self._close_socket_locked()
            self.sock = sock
            self.reader = threading.Thread(target=self._read_loop, args=(sock,), daemon=True)
            self.reader.start()
        print(f"Connected to Bluetooth gripper bridge at {self.host}:{self.port}")

    def close(self) -> None:
        self.stop_event.set()
        with self.socket_lock:
            self._close_socket_locked()

    def _close_socket_locked(self) -> None:
        sock = self.sock
        self.sock = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _disconnect_socket(self, sock: socket.socket | None = None) -> None:
        with self.socket_lock:
            if sock is not None and self.sock is not sock:
                return
            self._close_socket_locked()

    def _ensure_connected(self) -> None:
        if self.sock is not None:
            return
        self.connect()

    def _read_loop(self, sock: socket.socket) -> None:
        buffer = b""
        while not self.stop_event.is_set():
            try:
                data = sock.recv(4096)
            except OSError:
                break
            if not data:
                break
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    self.last_received_line = (time.monotonic(), text)
                    self.lines.put(text)
                    if self.trace_commands or "ERROR:" in text or "LOAD_STOP" in text:
                        print(f"ESP32> {text}")
        self._disconnect_socket(sock)

    def send(self, command: str) -> None:
        if self.trace_commands:
            print(f"> {command}")
        payload = (command + "\n").encode("ascii")
        self.last_command = command
        try:
            with self.socket_lock:
                if self.sock is None:
                    raise OSError("Bluetooth gripper bridge is not connected")
                self.sock.sendall(payload)
        except OSError as exc:
            self._disconnect_socket()
            raise ConnectionError(f"Send failed: {command}; not reconnected or replayed: {exc}") from exc

    def drain_lines(self) -> None:
        while True:
            try:
                self.lines.get_nowait()
            except queue.Empty:
                return

    def wait_for_command_ack(self, success_text: str, timeout: float = COMMAND_ACK_TIMEOUT_SEC) -> None:
        deadline = time.monotonic() + timeout
        replies = []
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=max(0.01, deadline - time.monotonic()))
            except queue.Empty:
                break
            replies.append(line)
            if "ERROR:" in line:
                raise RuntimeError(line)
            if success_text in line:
                return
        raise TimeoutError(
            f"No ESP32 acknowledgement for {success_text} within {timeout:.2f}s; "
            f"command={self.last_command!r}; tcp_connected={self.sock is not None}; "
            f"received={replies[-3:]!r}; last_rx={self.last_received_line!r}. "
            "Motion command was not resent."
        )

    def sync_move(self, raw1: int, raw2: int, speed: int, acc: int) -> None:
        logical1 = raw_to_logical(raw1)
        logical2 = raw_to_logical(raw2)
        if self.trace_commands:
            print(f"raw target: id1={raw1}, id2={raw2}")
        self.drain_lines()
        send_start_ns = time.monotonic_ns()
        self.send(f"SYNC_MOVE 1 {logical1} 2 {logical2} {speed} {acc}")
        self.wait_for_command_ack("SYNC_MOVE sent")
        self.last_move_timing = {"send_start_ns": send_start_ns, "ack_ns": time.monotonic_ns()}
        self.last_raws = (int(raw1), int(raw2))

    def hold(self) -> tuple[int, int]:
        self.drain_lines()
        self.send("HOLD")
        deadline = time.monotonic() + COMMAND_ACK_TIMEOUT_SEC
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=max(0.001, deadline - time.monotonic()))
            except queue.Empty:
                break
            if "ERROR:" in line:
                raise RuntimeError(line)
            match = re.fullmatch(r"HOLD sent: id1=(\d+) id2=(\d+)", line)
            if match:
                return tuple(int(v) for v in match.groups())
        raise TimeoutError("ESP32 HOLD not acknowledged; stop state unknown")

    def read_contact_positions(self, timeout: float = 0.5) -> tuple[int, int]:
        """Batch existing READ commands; defer telemetry callbacks until after stop."""
        self.drain_lines()
        self.send("READ 1\nREAD 2")
        deadline = time.monotonic() + timeout
        positions = {}
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=max(0.001, deadline - time.monotonic()))
            except queue.Empty:
                break
            if "ERROR:" in line:
                raise RuntimeError(line)
            match = POSITION_RE.search(line)
            if match and int(match.group("id")) in (1, 2):
                positions[int(match.group("id"))] = int(match.group("raw"))
                if len(positions) == 2:
                    return positions[1], positions[2]
        raise TimeoutError("Contact position batch incomplete; stop state unknown")

    def read_position(self, servo_id: int, timeout: float = 0.5) -> int | None:
        self.drain_lines()
        self.send(f"READ {servo_id}")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=max(0.01, deadline - time.monotonic()))
            except queue.Empty:
                break
            match = POSITION_RE.search(line)
            if match and int(match.group("id")) == servo_id:
                raw = int(match.group("raw"))
                callback = self.position_callback
                if callback is not None:
                    try:
                        callback(servo_id, raw)
                    except Exception as exc:
                        print(f"WARNING: gripper position callback failed: {exc}")
                return raw
            if f"ERROR: servo not found: ID {servo_id}" in line:
                return None
        return None


def format_strengths(strengths: list[float]) -> str:
    parts = []
    for value in strengths:
        if value != value:
            parts.append("lost")
        else:
            parts.append(f"{value:.1f}")
    return ", ".join(parts)


def config_raw_pair(config: dict, key: str, default: tuple[int, int]) -> tuple[int, int]:
    value = config.get(key)
    try:
        if isinstance(value, dict):
            return int(value["1"]), int(value["2"])
        if isinstance(value, list) and len(value) >= 2:
            return int(value[0]), int(value[1])
    except (KeyError, TypeError, ValueError):
        pass
    return default


def open_raws(config: dict) -> tuple[int, int]:
    return config_raw_pair(config, "open", DEFAULT_OPEN_RAWS)


def close_raws(config: dict) -> tuple[int, int]:
    return config_raw_pair(config, "close", DEFAULT_CLOSE_RAWS)


def interpolate_pair(
    start: tuple[int, int],
    target: tuple[int, int],
    ratio: float,
) -> tuple[int, int]:
    ratio = max(0.0, min(1.0, ratio))
    return (
        int(round(start[0] + (target[0] - start[0]) * ratio)),
        int(round(start[1] + (target[1] - start[1]) * ratio)),
    )


def rezero_anyskin(monitor: AnySkinMonitor, samples: int) -> None:
    if not monitor.enabled:
        return
    monitor.calibrate(samples)


def wait_for_open_pose(
    client: BluetoothGripperClient,
    target: tuple[int, int],
    tolerance_raw: int,
    timeout_sec: float,
    poll_sec: float = 0.08,
) -> bool:
    if timeout_sec <= 0.0:
        return False

    deadline = time.monotonic() + timeout_sec
    last_position: tuple[int | None, int | None] = (None, None)
    while time.monotonic() < deadline:
        position1 = client.read_position(1, timeout=0.25)
        position2 = client.read_position(2, timeout=0.25)
        last_position = (position1, position2)
        if (
            position1 is not None
            and position2 is not None
            and abs(position1 - target[0]) <= tolerance_raw
            and abs(position2 - target[1]) <= tolerance_raw
        ):
            print(f"Open pose reached: id1={position1}, id2={position2}.")
            return True
        time.sleep(poll_sec)

    print(
        "WARNING: Open pose was not confirmed before AnySkin rezero "
        f"(last id1={last_position[0]}, id2={last_position[1]}, "
        f"target id1={target[0]}, id2={target[1]})."
    )
    return False


def flatten_magnet_strengths(strengths: list[list[float]]) -> list[float]:
    values: list[float] = []
    for sensor_values in strengths:
        for value in sensor_values:
            if value is not None and not math.isnan(value):
                values.append(float(value))
    return values


def wait_for_anyskin_stability(
    monitor: AnySkinMonitor,
    stable_sec: float,
    change_threshold: float,
    timeout_sec: float,
    poll_sec: float = 0.05,
) -> bool:
    if not monitor.enabled or stable_sec <= 0.0 or timeout_sec <= 0.0:
        return True

    deadline = time.monotonic() + timeout_sec
    previous_values: list[float] | None = None
    stable_started_at: float | None = None
    last_delta = float("nan")

    while time.monotonic() < deadline:
        values = flatten_magnet_strengths(monitor.magnet_strengths())
        if not values:
            previous_values = None
            stable_started_at = None
            time.sleep(poll_sec)
            continue

        if previous_values is not None and len(previous_values) == len(values):
            last_delta = max(
                abs(current - previous)
                for current, previous in zip(values, previous_values)
            )
            if last_delta <= change_threshold:
                if stable_started_at is None:
                    stable_started_at = time.monotonic()
                if time.monotonic() - stable_started_at >= stable_sec:
                    print(
                        "AnySkin stabilized after open "
                        f"(max_delta={last_delta:.2f}); rezeroing."
                    )
                    return True
            else:
                stable_started_at = None
        previous_values = values
        time.sleep(poll_sec)

    delta_text = "unknown" if math.isnan(last_delta) else f"{last_delta:.2f}"
    print(
        "WARNING: AnySkin did not fully stabilize before rezero "
        f"(last max_delta={delta_text}); rezeroing anyway."
    )
    return False


def open_gripper(
    client: BluetoothGripperClient,
    monitor: AnySkinMonitor,
    config: dict,
    speed: int,
    acc: int,
    rezero_samples: int,
    rezero_delay_sec: float,
    position_tolerance_raw: int,
    settle_timeout_sec: float,
    stability_sec: float,
    stability_threshold: float,
) -> None:
    target = open_raws(config)
    client.sync_move(target[0], target[1], speed, acc)
    if monitor.enabled and rezero_samples > 0:
        reached = wait_for_open_pose(
            client,
            target,
            position_tolerance_raw,
            settle_timeout_sec,
        )
        if rezero_delay_sec > 0.0:
            time.sleep(rezero_delay_sec)
        wait_for_anyskin_stability(
            monitor,
            stability_sec,
            stability_threshold,
            settle_timeout_sec if reached else max(0.5, settle_timeout_sec),
        )
        rezero_anyskin(monitor, rezero_samples)
        print(f"AnySkin rezeroed after open with {rezero_samples} samples.")


def close_gripper(client: BluetoothGripperClient, config: dict, speed: int, acc: int) -> None:
    target = close_raws(config)
    client.sync_move(target[0], target[1], speed, acc)


def interpolate_raws(config: dict, ratio: float) -> tuple[int, int]:
    return interpolate_pair(open_raws(config), close_raws(config), ratio)


def baseline_has_per_magnet_values(config: dict) -> bool:
    baseline = config.get("empty_close_baseline", {})
    points = baseline.get("points", [])
    return bool(points and isinstance(points[0], dict) and "magnet_strengths" in points[0])


def baseline_matches_motion(config: dict) -> bool:
    baseline = config.get("empty_close_baseline")
    if not isinstance(baseline, dict):
        return False
    return (
        tuple(baseline.get("open", [])) == open_raws(config)
        and tuple(baseline.get("close", [])) == close_raws(config)
    )


def finite_strength(value: float | None) -> bool:
    return value is not None and not math.isnan(value)


CONTACT_MAGNET_GROUPS = (
    (0,),        # M1
    (1, 2, 3),  # M2/M3/M4 move together on this gripper, so count them once.
    (4,),        # M5
)


def grouped_contact_deltas(
    values: list[float],
    expected_values: list[float],
) -> list[float]:
    grouped: list[float] = []
    used: set[int] = set()
    for group in CONTACT_MAGNET_GROUPS:
        channel_deltas = []
        for mag_index in group:
            if mag_index >= len(values):
                continue
            baseline = expected_values[mag_index] if mag_index < len(expected_values) else 0.0
            value = values[mag_index]
            channel_deltas.append(
                float("nan") if not finite_strength(value) else value - baseline
            )
            used.add(mag_index)
        if channel_deltas:
            grouped.append(max_finite(channel_deltas))

    # If a future sensor exposes more than five channels, keep extras independent.
    for mag_index, value in enumerate(values):
        if mag_index in used:
            continue
        baseline = expected_values[mag_index] if mag_index < len(expected_values) else 0.0
        grouped.append(float("nan") if not finite_strength(value) else value - baseline)
    return grouped


def unknown_material_hint(note: str | None = None) -> dict:
    hint = dict(UNKNOWN_MATERIAL_HINT)
    hint["contact_confirmed"] = False
    if note:
        hint["note"] = note
    return hint


def classify_material_hint(
    deltas: list[float],
    delta_history: list[tuple[float, float, float]],
    contact_ratio: float,
    elapsed_sec: float,
    soft_delta: float,
    hard_delta: float,
    soft_rise_rate: float,
    hard_rise_rate: float,
    rise_window_sec: float,
) -> dict:
    finite_deltas = [float(value) for value in deltas if finite_strength(value)]
    if not finite_deltas:
        return unknown_material_hint("No finite AnySkin delta was available at contact.")

    peak_delta = max(finite_deltas)
    average_delta = sum(max(0.0, value) for value in finite_deltas) / len(finite_deltas)
    contact_ratio = max(0.0, min(1.0, float(contact_ratio)))
    rise_window_sec = max(0.03, rise_window_sec)
    recent_history = [
        sample
        for sample in delta_history
        if elapsed_sec - rise_window_sec <= sample[0] <= elapsed_sec
    ]
    if len(recent_history) < 2 and len(delta_history) >= 2:
        recent_history = delta_history[-min(8, len(delta_history)):]

    rise_rate = float("nan")
    if len(recent_history) >= 2:
        start_time, _, start_delta = recent_history[0]
        end_time, _, end_delta = recent_history[-1]
        duration = max(1e-6, end_time - start_time)
        rise_rate = max(0.0, end_delta - start_delta) / duration

    label = "soft"
    basis = "rise"
    note = "Soft-ish hint: AnySkin delta rose gradually during closing."
    if finite_strength(rise_rate) and rise_rate >= hard_rise_rate:
        label = "hard"
        note = "Hard-ish hint: AnySkin delta rose sharply at contact."
    elif (
        peak_delta >= hard_delta
        or (contact_ratio < 0.45 and peak_delta >= soft_delta)
    ):
        label = "hard"
        basis = "peak"
        note = "Hard-ish hint: contact was strong or early in the close motion."
    elif finite_strength(rise_rate) and rise_rate <= soft_rise_rate:
        label = "soft"
        note = "Soft-ish hint: AnySkin delta rose slowly after contact."
    elif peak_delta < soft_delta or (contact_ratio > 0.78 and peak_delta < hard_delta):
        label = "soft"
        basis = "peak"
        note = "Soft-ish hint: contact stayed mild or appeared late in the close motion."

    return {
        "label": label,
        "contact_confirmed": True,
        "peak_delta": round(peak_delta, 2),
        "average_delta": round(average_delta, 2),
        "rise_rate": None if not finite_strength(rise_rate) else round(rise_rate, 2),
        "contact_ratio": round(contact_ratio, 3),
        "elapsed_sec": round(float(elapsed_sec), 3),
        "classification_basis": basis,
        "note": note,
    }


def print_material_hint(hint: dict) -> None:
    label = hint.get("label", "unknown")
    peak_delta = hint.get("peak_delta")
    rise_rate = hint.get("rise_rate")
    contact_ratio = hint.get("contact_ratio")
    elapsed_sec = hint.get("elapsed_sec")
    print(
        "Material hint: "
        f"{label}, peak_delta={peak_delta}, "
        f"rise_rate={rise_rate}, "
        f"contact_ratio={contact_ratio}, elapsed_sec={elapsed_sec}"
    )


def percentile_value(values: list[float], percentile: float) -> float:
    finite_values = sorted(value for value in values if finite_strength(value))
    if not finite_values:
        return float("nan")
    percentile = max(0.0, min(100.0, percentile))
    if len(finite_values) == 1:
        return finite_values[0]
    position = (len(finite_values) - 1) * (percentile / 100.0)
    lower_index = int(math.floor(position))
    upper_index = int(math.ceil(position))
    if lower_index == upper_index:
        return finite_values[lower_index]
    lower = finite_values[lower_index]
    upper = finite_values[upper_index]
    return lower + (upper - lower) * (position - lower_index)


def sample_empty_strength_envelope(
    monitor: AnySkinMonitor,
    samples: int,
    percentile: float,
    sample_delay_sec: float,
) -> list[list[float]]:
    """Sample the no-object gripper motion envelope per sensor and magnet."""
    samples = max(1, samples)
    sensor_count = len(monitor.ports)
    buckets = [
        [[] for _ in range(monitor.num_mags)]
        for _ in range(sensor_count)
    ]
    for _ in range(samples):
        magnet_strengths = monitor.magnet_strengths()
        for sensor_index in range(sensor_count):
            values = magnet_strengths[sensor_index] if sensor_index < len(magnet_strengths) else []
            for mag_index in range(monitor.num_mags):
                value = values[mag_index] if mag_index < len(values) else float("nan")
                if finite_strength(value):
                    buckets[sensor_index][mag_index].append(float(value))
        time.sleep(max(0.0, sample_delay_sec))

    return [
        [
            percentile_value(mag_values, percentile)
            for mag_values in sensor_values
        ]
        for sensor_values in buckets
    ]


def merge_empty_strength_envelopes(
    envelopes: list[list[list[float]]],
    percentile: float,
    sensor_count: int,
    num_mags: int,
) -> list[list[float]]:
    merged: list[list[float]] = []
    for sensor_index in range(sensor_count):
        row: list[float] = []
        for mag_index in range(num_mags):
            values = []
            for envelope in envelopes:
                if sensor_index >= len(envelope):
                    continue
                sensor_values = envelope[sensor_index]
                if mag_index >= len(sensor_values):
                    continue
                value = sensor_values[mag_index]
                if finite_strength(value):
                    values.append(float(value))
            row.append(percentile_value(values, percentile))
        merged.append(row)
    return merged


def hold_current_position(
    client: BluetoothGripperClient,
    hold_speed: int,
    hold_acc: int,
) -> bool:
    position1 = client.read_position(1)
    position2 = client.read_position(2)
    if position1 is None or position2 is None:
        print("WARNING: Could not read current gripper pose; torque off fallback.")
        client.send("TORQUE_OFF 1")
        client.send("TORQUE_OFF 2")
        return False
    client.sync_move(position1, position2, hold_speed, hold_acc)
    print(f"AnySkin stop: holding current pose id1={position1}, id2={position2}")
    return True


def hold_raw_position(
    client: BluetoothGripperClient,
    raws: tuple[int, int],
    hold_speed: int,
    hold_acc: int,
) -> None:
    client.sync_move(raws[0], raws[1], hold_speed, hold_acc)
    print(f"AnySkin stop: holding commanded pose id1={raws[0]}, id2={raws[1]}")


def hold_gentle_contact_position(
    client: BluetoothGripperClient,
    stop_raws: tuple[int, int],
    open_target: tuple[int, int],
    backoff_raw: int,
    hold_speed: int,
    hold_acc: int,
) -> None:
    # Stop the in-flight close immediately. Reading both servos first can take
    # hundreds of milliseconds, which is long enough to squeeze a soft object.
    client.sync_move(stop_raws[0], stop_raws[1], hold_speed, hold_acc)
    print(
        "AnySkin stop: immediate hold at last commanded pose "
        f"id1={stop_raws[0]}, id2={stop_raws[1]}."
    )
    current1 = client.read_position(1, timeout=0.2)
    current2 = client.read_position(2, timeout=0.2)
    if current1 is not None and current2 is not None:
        stop_raws = (current1, current2)
        print(
            "AnySkin stop: contact pose read before hold "
            f"id1={current1}, id2={current2}."
        )
    else:
        print(
            "WARNING: Could not read contact pose quickly; "
            f"holding from last commanded pose id1={stop_raws[0]}, id2={stop_raws[1]}."
        )
    hold_raws = tuple(backoff_toward_open(list(stop_raws), list(open_target), backoff_raw))
    client.sync_move(hold_raws[0], hold_raws[1], hold_speed, hold_acc)
    print(
        "AnySkin stop: holding contact pose. "
        f"raw={hold_raws}, backoff={backoff_raw}"
    )


def calibrate_empty_close_baseline(
    client: BluetoothGripperClient,
    monitor: AnySkinMonitor,
    config: dict,
    config_path: Path,
    speed: int,
    acc: int,
    calibration_samples: int,
    steps: int,
    step_interval_sec: float,
    point_samples: int,
    baseline_percentile: float,
    cycles: int,
    cancel_requested: Callable[[], bool] | None = None,
) -> None:
    if not monitor.enabled:
        print("AnySkin is disabled. Start with --anyskin-ports <port1,port2>.")
        return
    steps = max(1, steps)
    open_target = open_raws(config)
    close_target = close_raws(config)
    def check_cancelled():
        if cancel_requested is not None and cancel_requested():
            raise RuntimeError("Empty-close calibration cancelled")

    check_cancelled()
    print("Calibrating empty close baseline. Keep the gripper empty.")
    client.sync_move(open_target[0], open_target[1], speed, acc)
    time.sleep(0.6)
    monitor.calibrate(max(80, calibration_samples))

    cycles = max(1, cycles)
    point_samples = max(1, point_samples)
    point_envelopes: list[list[list[list[float]]]] = [
        []
        for _ in range(steps + 1)
    ]

    for cycle_index in range(cycles):
        check_cancelled()
        if cycle_index > 0:
            client.sync_move(open_target[0], open_target[1], speed, acc)
            time.sleep(max(0.25, step_interval_sec * 2.0))
        print(f"Empty close calibration cycle {cycle_index + 1}/{cycles}")
        for step_index in range(steps + 1):
            check_cancelled()
            ratio = step_index / steps
            target = interpolate_pair(open_target, close_target, ratio)
            client.sync_move(target[0], target[1], speed, acc)
            time.sleep(max(step_interval_sec, 0.03))
            magnet_strengths = sample_empty_strength_envelope(
                monitor,
                point_samples,
                baseline_percentile,
                min(0.02, max(0.006, step_interval_sec * 0.5)),
            )
            point_envelopes[step_index].append(magnet_strengths)
            strengths = sensor_max_strengths(magnet_strengths)
            print(
                f"  cycle={cycle_index + 1} ratio={ratio:.2f} "
                f"raws={target} empty_strength={format_strengths(strengths)}"
            )

    points = []
    for step_index, envelopes in enumerate(point_envelopes):
        ratio = step_index / steps
        target = interpolate_pair(open_target, close_target, ratio)
        magnet_strengths = merge_empty_strength_envelopes(
            envelopes,
            baseline_percentile,
            len(monitor.ports),
            monitor.num_mags,
        )
        strengths = sensor_max_strengths(magnet_strengths)
        points.append(
            {
                "ratio": ratio,
                "raws": [target[0], target[1]],
                "magnet_strengths": [
                    [
                        None if not finite_strength(value) else float(value)
                        for value in sensor_values
                    ]
                    for sensor_values in magnet_strengths
                ],
                "strengths": [
                    None if not finite_strength(value) else float(value)
                    for value in strengths
                ],
            }
        )
        print(
            f"  saved ratio={ratio:.2f} raws={target} "
            f"empty_envelope={format_strengths(strengths)}"
        )

    check_cancelled()
    config["empty_close_baseline"] = {
        "open": list(open_target),
        "close": list(close_target),
        "sensor_ports": monitor.ports,
        "num_mags": monitor.num_mags,
        "calibration_baselines": monitor.calibration_snapshot(),
        "point_samples": max(1, point_samples),
        "baseline_percentile": max(0.0, min(100.0, baseline_percentile)),
        "cycles": cycles,
        "points": points,
    }
    save_config(config_path, config)
    check_cancelled()
    client.sync_move(open_target[0], open_target[1], speed, acc)
    print(
        "Saved empty close baseline envelope "
        f"with {len(points)} points, {cycles} cycle(s), "
        f"{max(1, point_samples)} samples/point, "
        f"p{max(0.0, min(100.0, baseline_percentile)):.0f} to {config_path}."
    )


def close_safe(
    client: BluetoothGripperClient,
    monitor: AnySkinMonitor,
    config: dict,
    speed: int,
    acc: int,
    confirm_sec: float,
    poll_sec: float,
    timeout_sec: float,
    hold_speed: int,
    hold_acc: int,
    print_interval_sec: float,
    min_close_sec: float,
    baseline_margin: float,
    confirm_samples: int,
    contact_min_channels: int,
    contact_min_sensors: int,
    single_channel_strong_delta: float,
    min_contact_ratio: float,
    steps: int,
    step_interval_sec: float,
    stop_backoff_raw: int,
    material_soft_delta: float,
    material_hard_delta: float,
    material_soft_rise_rate: float,
    material_hard_rise_rate: float,
    material_rise_window_sec: float,
    allow_unbaselined: bool,
    cancel_requested: Callable[[], bool] | None = None,
) -> dict:
    if not monitor.enabled:
        print("AnySkin is disabled. Start with --anyskin-ports <port1,port2>.")
        return unknown_material_hint("AnySkin is disabled.")

    has_empty_baseline = bool(config.get("empty_close_baseline"))
    has_per_magnet_baseline = has_empty_baseline and baseline_has_per_magnet_values(config)
    has_matching_baseline = has_empty_baseline and baseline_matches_motion(config)
    baseline_problem = None
    if not has_empty_baseline:
        baseline_problem = "empty close baseline is missing"
    elif not has_per_magnet_baseline:
        baseline_problem = "empty close baseline is in the old per-sensor format"
    elif not has_matching_baseline:
        baseline_problem = "empty close baseline was recorded for a different open/close range"
    if baseline_problem:
        message = (
            f"{baseline_problem}. Run open, then empty_close_calibrate with no object "
            "before using close_safe."
        )
        if not allow_unbaselined:
            print(f"ERROR: {message}")
            return unknown_material_hint(message)
        print(f"WARNING: {message} Continuing because --allow-unbaselined-close-safe was set.")

    print(
        "close_safe started. Contact must stay above threshold/empty baseline "
        f"for {confirm_sec:.2f}s."
    )
    open_target = open_raws(config)
    close_target = close_raws(config)
    position1 = client.read_position(1, timeout=0.35)
    position2 = client.read_position(2, timeout=0.35)
    if position1 is not None and position2 is not None:
        start_target = (position1, position2)
    elif client.last_raws is not None:
        start_target = client.last_raws
        print(
            "WARNING: Could not read gripper pose; using last commanded pose "
            f"id1={start_target[0]}, id2={start_target[1]}."
        )
    else:
        start_target = open_target
        print("WARNING: Could not read gripper pose; assuming saved open pose.")

    confirm_samples = max(1, confirm_samples)
    contact_min_channels = max(1, contact_min_channels)
    available_contact_sensors = max(
        1,
        len(
            [
                index
                for index in range(len(monitor.ports))
                if index not in monitor.ignored_indexes
            ]
        ),
    )
    contact_min_sensors = max(
        1,
        min(contact_min_sensors, available_contact_sensors),
    )
    single_channel_strong_delta = max(baseline_margin, single_channel_strong_delta)
    min_contact_ratio = min(1.0, max(0.0, min_contact_ratio))
    print(
        "Contact filter: "
        "M2/M3/M4 are grouped as one contact channel; "
        f"{contact_min_channels}+ sustained contact channel(s), or one channel "
        f"above {single_channel_strong_delta:.1f}, after close ratio "
        f"{min_contact_ratio:.2f}; {contact_min_sensors}+ sensor(s) must respond."
    )
    contact_started_at_by_channel: list[list[float | None]] = []
    contact_counts_by_channel: list[list[int]] = []
    started_at = time.monotonic()
    last_print = 0.0
    last_commanded = start_target
    delta_history: list[tuple[float, float, float]] = []

    def cancelled() -> bool:
        if cancel_requested is None or not cancel_requested():
            return False
        print("close_safe cancelled by open command.")
        return True

    def required_sensor_data_lost(strengths: list[float]) -> bool:
        return any(
            sensor_index >= len(strengths)
            or not finite_strength(strengths[sensor_index])
            for sensor_index in range(len(monitor.ports))
            if sensor_index not in monitor.ignored_indexes
        )

    def stop_for_sensor_loss(strengths: list[float]) -> dict | None:
        if not required_sensor_data_lost(strengths):
            return None
        message = (
            "Required AnySkin data was lost while closing; holding the gripper "
            "at the last commanded pose."
        )
        print(f"ERROR: {message}")
        hold_gentle_contact_position(
            client,
            last_commanded,
            open_target,
            0,
            hold_speed,
            hold_acc,
        )
        return unknown_material_hint(message)

    def record_delta_sample(elapsed: float, ratio: float, deltas: list[float]) -> None:
        peak_delta = max_finite(deltas)
        if not finite_strength(peak_delta):
            return
        delta_history.append((elapsed, ratio, max(0.0, float(peak_delta))))
        keep_after = elapsed - max(1.0, material_rise_window_sec * 3.0)
        while delta_history and delta_history[0][0] < keep_after:
            delta_history.pop(0)

    def confirmed_contact(ratio: float) -> tuple[bool, list[float], list[float]]:
        nonlocal contact_started_at_by_channel, contact_counts_by_channel
        magnet_strengths = monitor.magnet_strengths()
        strengths = sensor_max_strengths(magnet_strengths)
        expected = empty_baseline_magnet_strengths(
            config,
            ratio,
            len(magnet_strengths),
            monitor.num_mags,
        )
        now = time.monotonic()
        deltas: list[float] = []
        grouped_deltas_by_sensor: list[list[float]] = []
        mature_deltas: list[float] = []
        mature_sensor_indexes: set[int] = set()
        allow_contact = ratio >= min_contact_ratio
        for sensor_index, values in enumerate(magnet_strengths):
            expected_values = expected[sensor_index] if sensor_index < len(expected) else []
            sensor_deltas = []
            for mag_index, value in enumerate(values):
                baseline = expected_values[mag_index] if mag_index < len(expected_values) else 0.0
                delta = float("nan") if not finite_strength(value) else value - baseline
                sensor_deltas.append(delta)
            deltas.append(max_finite(sensor_deltas))
            grouped_deltas_by_sensor.append(
                grouped_contact_deltas(values, expected_values)
            )

        grouped_shape = [len(values) for values in grouped_deltas_by_sensor]
        if [len(values) for values in contact_started_at_by_channel] != grouped_shape:
            contact_started_at_by_channel = [
                [None] * len(values)
                for values in grouped_deltas_by_sensor
            ]
            contact_counts_by_channel = [
                [0] * len(values)
                for values in grouped_deltas_by_sensor
            ]

        for sensor_index, channel_deltas in enumerate(grouped_deltas_by_sensor):
            for channel_index, delta in enumerate(channel_deltas):
                candidate = (
                    allow_contact
                    and sensor_index not in monitor.ignored_indexes
                    and finite_strength(delta)
                    and delta >= baseline_margin
                )
                if candidate:
                    if contact_started_at_by_channel[sensor_index][channel_index] is None:
                        contact_started_at_by_channel[sensor_index][channel_index] = now
                        contact_counts_by_channel[sensor_index][channel_index] = 0
                    contact_counts_by_channel[sensor_index][channel_index] += 1
                    if (
                        now - contact_started_at_by_channel[sensor_index][channel_index] >= confirm_sec
                        and contact_counts_by_channel[sensor_index][channel_index] >= confirm_samples
                    ):
                        mature_deltas.append(delta)
                        mature_sensor_indexes.add(sensor_index)
                else:
                    contact_started_at_by_channel[sensor_index][channel_index] = None
                    contact_counts_by_channel[sensor_index][channel_index] = 0

        channel_confirmed = (
            len(mature_deltas) >= contact_min_channels
            or (
                len(mature_deltas) >= 1
                and max(mature_deltas) >= single_channel_strong_delta
            )
        )
        confirmed = (
            channel_confirmed
            and len(mature_sensor_indexes) >= contact_min_sensors
        )
        return confirmed, strengths, deltas

    def contact_pending() -> bool:
        return sum(
            any(stamp is not None for stamp in channels)
            for channels in contact_started_at_by_channel
        ) >= contact_min_sensors

    steps = max(1, steps)
    for step_index in range(1, steps + 1):
        if cancelled():
            return unknown_material_hint("Close Safe was cancelled by an open command.")
        if time.monotonic() - started_at >= timeout_sec:
            break
        previous_ratio = close_ratio_from_raws(last_commanded, open_target, close_target)
        contact, strengths, deltas = confirmed_contact(previous_ratio)
        sensor_loss_hint = stop_for_sensor_loss(strengths)
        if sensor_loss_hint is not None:
            return sensor_loss_hint
        # Keep the current target while debouncing a possible contact.
        while contact_pending() and not contact:
            if cancelled():
                return unknown_material_hint("Close Safe was cancelled by an open command.")
            if time.monotonic() - started_at >= timeout_sec:
                return unknown_material_hint("Close Safe timed out while confirming contact.")
            time.sleep(poll_sec)
            contact, strengths, deltas = confirmed_contact(previous_ratio)
            sensor_loss_hint = stop_for_sensor_loss(strengths)
            if sensor_loss_hint is not None:
                return sensor_loss_hint
        now = time.monotonic()
        elapsed = now - started_at
        record_delta_sample(elapsed, previous_ratio, deltas)
        if now - last_print >= print_interval_sec:
            print(
                "AnySkin strength: "
                f"{format_strengths(strengths)} delta={format_strengths(deltas)} "
                f"contact={contact}"
            )
            last_print = now
        if elapsed >= min_close_sec and contact:
            hint = classify_material_hint(
                deltas,
                delta_history,
                previous_ratio,
                elapsed,
                material_soft_delta,
                material_hard_delta,
                material_soft_rise_rate,
                material_hard_rise_rate,
                material_rise_window_sec,
            )
            print_material_hint(hint)
            hold_gentle_contact_position(
                client,
                last_commanded,
                open_target,
                stop_backoff_raw,
                hold_speed,
                hold_acc,
            )
            return hint

        ratio = step_index / steps
        target = interpolate_pair(start_target, close_target, ratio)
        baseline_ratio = close_ratio_from_raws(target, open_target, close_target)
        client.sync_move(target[0], target[1], speed, acc)
        last_commanded = target

        deadline = min(time.monotonic() + step_interval_sec, started_at + timeout_sec)
        while (
            time.monotonic() < deadline or contact_pending()
        ) and time.monotonic() < started_at + timeout_sec:
            if cancelled():
                return unknown_material_hint("Close Safe was cancelled by an open command.")
            contact, strengths, deltas = confirmed_contact(baseline_ratio)
            sensor_loss_hint = stop_for_sensor_loss(strengths)
            if sensor_loss_hint is not None:
                return sensor_loss_hint
            now = time.monotonic()
            elapsed = now - started_at
            record_delta_sample(elapsed, baseline_ratio, deltas)
            if now - last_print >= print_interval_sec:
                print(
                    "AnySkin strength: "
                    f"{format_strengths(strengths)} delta={format_strengths(deltas)} "
                    f"contact={contact}"
                )
                last_print = now
            if elapsed < min_close_sec:
                contact_started_at_by_channel = [
                    [None] * len(values)
                    for values in contact_started_at_by_channel
                ]
                contact_counts_by_channel = [
                    [0] * len(values)
                    for values in contact_counts_by_channel
                ]
                time.sleep(poll_sec)
                continue
            if contact:
                hint = classify_material_hint(
                    deltas,
                    delta_history,
                    baseline_ratio,
                    elapsed,
                    material_soft_delta,
                    material_hard_delta,
                    material_soft_rise_rate,
                    material_hard_rise_rate,
                    material_rise_window_sec,
                )
                print_material_hint(hint)
                hold_gentle_contact_position(
                    client,
                    last_commanded,
                    open_target,
                    stop_backoff_raw,
                    hold_speed,
                    hold_acc,
                )
                return hint
            time.sleep(poll_sec)

    print("close_safe timeout reached without confirmed AnySkin contact.")
    return unknown_material_hint("Close Safe timed out without confirmed AnySkin contact.")


class WebGripperCommands:
    def __init__(
        self,
        client: BluetoothGripperClient,
        monitor: AnySkinMonitor,
        args: argparse.Namespace,
    ) -> None:
        self.client = client
        self.monitor = monitor
        self.args = args
        self.lock = threading.Lock()
        self.message = "commands ready"
        self.material_hint = unknown_material_hint()

    def status(self) -> dict:
        config = self.args.config_data
        open_target = open_raws(config)
        close_target = close_raws(config)
        last_raws = self.client.last_raws or open_target
        close_ratio = close_ratio_from_raws(last_raws, open_target, close_target)
        baseline_available = (
            baseline_has_per_magnet_values(config)
            and baseline_matches_motion(config)
        )
        expected_magnet_strengths = []
        if baseline_available:
            expected_magnet_strengths = empty_baseline_magnet_strengths(
                config,
                close_ratio,
                len(self.monitor.ports),
                self.monitor.num_mags,
            )
        if baseline_available:
            rule_description = (
                "empty-close baseline + "
                f"{self.args.safe_empty_baseline_margin:.1f} margin "
                f"@ {close_ratio * 100:.0f}% close"
            )
        else:
            rule_description = (
                "fixed threshold; run Open, then Calibrate Empty Close "
                "to show per-sensor dynamic baseline"
            )

        return {
            "ok": True,
            "busy": self.lock.locked(),
            "message": self.message,
            "material_hint": self.material_hint,
            "close_ratio": close_ratio,
            "empty_baseline_available": baseline_available,
            "empty_baseline_margin": self.args.safe_empty_baseline_margin,
            "expected_magnet_strengths": expected_magnet_strengths,
            "rule_description": rule_description,
        }

    def submit(self, command: str) -> dict:
        if command not in {"open", "close", "close_safe", "empty_close_calibrate"}:
            return {"ok": False, "message": f"unknown command: {command}"}
        if not self.lock.acquire(blocking=False):
            return {"ok": False, "message": "gripper command already running"}

        thread = threading.Thread(
            target=self._run,
            args=(command,),
            daemon=True,
        )
        thread.start()
        self.message = f"{command} started"
        return {"ok": True, "message": self.message}

    def _run(self, command: str, cancel_requested=None) -> None:
        try:
            self.message = f"{command} running"
            completion_message = f"{command} complete"
            if command == "open":
                open_gripper(
                    self.client,
                    self.monitor,
                    self.args.config_data,
                    self.args.speed,
                    self.args.acc,
                    self.args.open_rezero_samples,
                    self.args.open_rezero_delay_sec,
                    self.args.open_position_tolerance_raw,
                    self.args.open_settle_timeout_sec,
                    self.args.open_stability_sec,
                    self.args.open_stability_threshold,
                )
                self.material_hint = unknown_material_hint("Opened. Run Close Safe to estimate object stiffness.")
            elif command == "close":
                close_gripper(self.client, self.args.config_data, self.args.speed, self.args.acc)
                self.material_hint = unknown_material_hint("Plain close does not estimate object stiffness.")
            elif command == "empty_close_calibrate":
                calibrate_empty_close_baseline(
                    self.client,
                    self.monitor,
                    self.args.config_data,
                    self.args.config,
                    self.args.speed,
                    self.args.acc,
                    self.args.calibration_samples,
                    self.args.empty_close_steps,
                    self.args.empty_close_step_sec,
                    self.args.empty_close_point_samples,
                    self.args.empty_close_baseline_percentile,
                    self.args.empty_close_cycles,
                    cancel_requested,
                )
                self.material_hint = unknown_material_hint("Empty close baseline updated. Run Close Safe on an object.")
            elif command == "close_safe":
                self.material_hint = close_safe(
                    self.client,
                    self.monitor,
                    self.args.config_data,
                    self.args.speed,
                    self.args.acc,
                    self.args.safe_contact_confirm_sec,
                    self.args.safe_poll_sec,
                    self.args.safe_timeout_sec,
                    self.args.hold_speed,
                    self.args.hold_acc,
                    self.args.print_interval_sec,
                    self.args.safe_min_close_sec,
                    self.args.safe_empty_baseline_margin,
                    self.args.safe_contact_confirm_samples,
                    self.args.safe_contact_min_channels,
                    self.args.safe_contact_min_sensors,
                    self.args.safe_single_channel_strong_delta,
                    self.args.safe_min_contact_ratio,
                    self.args.safe_close_steps,
                    self.args.safe_close_step_sec,
                    self.args.safe_stop_backoff_raw,
                    self.args.material_soft_delta,
                    self.args.material_hard_delta,
                    self.args.material_soft_rise_rate,
                    self.args.material_hard_rise_rate,
                    self.args.material_rise_window_sec,
                    self.args.allow_unbaselined_close_safe,
                )
                completion_message = (
                    "close_safe complete: "
                    f"{self.material_hint.get('label', 'unknown')} object hint"
                )
            self.message = completion_message
        except Exception as exc:
            self.message = f"{command} failed: {exc}"
        finally:
            self.lock.release()


def start_web_server(
    monitor: AnySkinMonitor,
    host: str,
    port: int,
    command_handler=None,
    command_status_handler=None,
) -> ThreadingHTTPServer:
    AnySkinWebHandler.monitor = monitor
    AnySkinWebHandler.command_handler = command_handler
    AnySkinWebHandler.command_status_handler = command_status_handler
    server = ThreadingHTTPServer((host, port), AnySkinWebHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"AnySkin web monitor: http://{host}:{port}")
    return server


class RosDiscreteGripperController:
    """Turn GELLO open/close width commands into open and close_safe actions."""

    def __init__(
        self,
        topic: str,
        input_min_width: float,
        input_max_width: float,
        open_action: Callable[[], None],
        close_action: Callable[[Callable[[], bool]], dict],
        feedback_topic: str,
        actual_position_topic: str,
    ) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Bool, Float64MultiArray, String
        except ImportError as exc:
            raise RuntimeError(
                "ROS command mode requires ROS 2 Python packages. "
                "Run source /opt/ros/humble/setup.bash first."
            ) from exc

        self.rclpy = rclpy
        self.owns_rclpy_context = not rclpy.ok()
        if self.owns_rclpy_context:
            rclpy.init()
        self.node = Node("ur3_anyskin_gripper_controller")
        self.input_min_width = float(input_min_width)
        self.input_max_width = float(input_max_width)
        self.open_action = open_action
        self.close_action = close_action
        self.Bool = Bool
        self.Float64MultiArray = Float64MultiArray
        self.String = String
        self.timing_publisher = self.node.create_publisher(String, "/gello/control_timing", 100)
        self.feedback_active = False
        self.desired_state: str | None = None
        self.completed_state: str | None = None
        self.stop_event = threading.Event()
        self.condition = threading.Condition()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.subscription = self.node.create_subscription(
            Float64MultiArray, topic, self._command_callback, 10
        )
        self.feedback_publisher = self.node.create_publisher(Bool, feedback_topic, 10)
        self.actual_position_publisher = self.node.create_publisher(
            Float64MultiArray, actual_position_topic, 10
        )
        self.feedback_timer = self.node.create_timer(0.2, self._publish_feedback)
        self.worker.start()
        self.spin_thread.start()
        self.node.get_logger().info(
            f"Listening for GELLO gripper commands on {topic}. "
            f"Confirmed contact is published on {feedback_topic}."
        )

    def publish_actual_position(self, servo_id: int, raw: int) -> None:
        message = self.Float64MultiArray()
        message.data = [float(servo_id), float(raw)]
        self.actual_position_publisher.publish(message)

    def _publish_feedback(self) -> None:
        with self.condition:
            active = self.feedback_active
        message = self.Bool()
        message.data = active
        self.feedback_publisher.publish(message)

    def _set_feedback(self, active: bool) -> None:
        with self.condition:
            changed = self.feedback_active != bool(active)
            self.feedback_active = bool(active)
        self._publish_feedback()
        if changed:
            state = "hold" if active else "released"
            self.node.get_logger().info(f"GELLO gripper feedback {state}.")

    def _command_callback(self, message) -> None:
        if not message.data:
            return
        width = float(message.data[0])
        midpoint = (self.input_min_width + self.input_max_width) * 0.5
        state = "open" if width >= midpoint else "close_safe"
        if state == "open":
            self._set_feedback(False)
        with self.condition:
            if state == self.desired_state:
                return
            self.desired_state = state
            self.condition.notify_all()
        self.node.get_logger().info(
            f"GELLO requested {state} (width={width:.4f} m)."
        )

    def _open_requested(self) -> bool:
        with self.condition:
            return self.desired_state == "open" or self.stop_event.is_set()

    def _worker_loop(self) -> None:
        while not self.stop_event.is_set():
            with self.condition:
                self.condition.wait_for(
                    lambda: self.stop_event.is_set()
                    or (
                        self.desired_state is not None
                        and self.desired_state != self.completed_state
                    ),
                    timeout=0.2,
                )
                if self.stop_event.is_set():
                    return
                state = self.desired_state
            try:
                if state == "open":
                    self._set_feedback(False)
                    self.open_action()
                elif state == "close_safe":
                    result = self.close_action(self._open_requested)
                    self._set_feedback(bool(result.get("contact_confirmed", False)))
            except Exception as exc:
                self._set_feedback(False)
                self.node.get_logger().error(f"{state} failed: {exc}")
                # Do not repeatedly retry a failed close without a new lever
                # transition. That could resume squeezing after a sensor or
                # Bluetooth fault.
                with self.condition:
                    if self.desired_state == state:
                        self.completed_state = state
                continue
            with self.condition:
                if self.desired_state == state:
                    self.completed_state = state

    def _spin_loop(self) -> None:
        try:
            self.rclpy.spin(self.node)
        except Exception as exc:
            if not self.stop_event.is_set():
                print(f"ROS gripper command listener stopped: {exc}")

    def submit(self, command: str) -> dict:
        if command not in {"open", "close_safe"}:
            return {
                "ok": False,
                "message": "ROS control mode accepts only open and close_safe",
            }
        with self.condition:
            self.desired_state = command
            self.completed_state = None
            self.condition.notify_all()
        return {"ok": True, "message": f"{command} requested"}

    def close(self) -> None:
        self._set_feedback(False)
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self.owns_rclpy_context and self.rclpy.ok():
            self.rclpy.shutdown()
        self.worker.join(timeout=1.0)
        self.spin_thread.join(timeout=1.0)


class RosPositionGripperController(RosDiscreteGripperController):
    """One worker owns Bluetooth; callbacks only replace the latest target."""

    def __init__(self, client, monitor, args, web_commands):
        self.client = client
        self.monitor = monitor
        self.args = args
        self.web_commands = web_commands
        self.tracker = None
        self.manual_command = None
        self.manual_anchor = None
        self.latest_ratio = None
        self.latest_received = None
        self.web_target_active = False
        self.failure = None
        self.startup_calibration_ready = not getattr(args, 'require_startup_calibration', False)
        self.await_open_after_calibration = False
        self.motion_timing = GripperMotionTiming()
        self.next_position_poll = 0.0
        self.position_poll_id = 1
        self.open_rezero_done = False
        self.open_rezero_check_at = 0.0
        self.message = "reading gripper position"
        if args.ros_input_max_width <= args.ros_input_min_width:
            raise ValueError("ROS input maximum width must exceed minimum width")
        super().__init__(args.ros_command_topic, args.ros_input_min_width,
                         args.ros_input_max_width, lambda: None, lambda cancel: {},
                         args.ros_feedback_topic, args.ros_actual_position_topic)
        self.node.get_logger().info("Continuous gripper position tracking enabled.")

    def _command_callback(self, message):
        if self.failure:
            return
        if not message.data or not math.isfinite(float(message.data[0])):
            return
        ratio = 1.0 - (float(message.data[0]) - self.input_min_width) / (
            self.input_max_width - self.input_min_width)
        with self.condition:
            ratio = min(1.0, max(0.0, ratio))
            if not self.startup_calibration_ready:
                return
            if self.await_open_after_calibration:
                if ratio > 0.003:
                    return
                self.await_open_after_calibration = False
                self.manual_anchor = None
            if self.manual_anchor is not None:
                if abs(ratio - self.manual_anchor) < 0.02:
                    return
                self.manual_anchor = None
            self.web_target_active = False
            self.latest_ratio = ratio
            self.latest_received = time.monotonic()

    def submit(self, command):
        if self.failure:
            return {"ok": False, "message": self.message}
        if command not in {"open", "close_safe", "empty_close_calibrate"}:
            return {"ok": False, "message": "Unsupported position tracking command"}
        if not self.startup_calibration_ready and command != "empty_close_calibrate":
            return {"ok": False, "message": "Clear gripper, then Calibrate Empty Close before control"}
        if self.await_open_after_calibration and command != "empty_close_calibrate":
            return {"ok": False, "message": "Fully open controller lever to enable control"}
        with self.condition:
            if self.manual_command is not None:
                return {"ok": False, "message": "Calibration already requested"}
            self.manual_anchor = self.latest_ratio
            if command == "empty_close_calibrate":
                self.manual_command = command
            else:
                self.web_target_active = True
                self.latest_ratio = 0.0 if command == "open" else 1.0
                self.latest_received = time.monotonic()
        return {"ok": True, "message": f"{command} requested"}

    def status(self):
        status = self.web_commands.status()
        status["message"] = self.message
        status["tracking_state"] = self.message
        status["contact_confirmed"] = bool(self.tracker and self.tracker.contact)
        status["motion_inhibited"] = bool(self.failure)
        status["fault"] = self.failure
        guard = getattr(self, 'regrasp', None)
        status['regrasp_enabled'] = getattr(self.args, 'slip_regrasp', 'false') == 'true'
        status['regrasp_state'] = guard.state if guard else 'not initialized'
        status['regrasp_attempts'] = guard.attempts if guard else 0
        status['regrasp_change_by_sensor'] = guard.change_by_sensor if guard else []
        status['regrasp_direction_by_sensor'] = guard.direction_by_sensor if guard else []
        status['regrasp_change_threshold'] = 20
        status['regrasp_range_low'] = guard.range_low if guard else None
        status['regrasp_range_high'] = guard.reference if guard else None
        status['regrasp_response_reference'] = guard.response_reference if guard else None
        status['slip_model_input'] = self.monitor.slip_model_snapshot()
        return status

    def _initial_tracker(self):
        self.regrasp = RegraspGuard()
        self.regrasp_origin = None
        self.client.position_callback = self.publish_actual_position
        positions = {}
        for attempt in range(3):
            for servo_id in (1, 2):
                if servo_id not in positions:
                    raw = self.client.read_position(servo_id, timeout=1.5)
                    if raw is not None:
                        positions[servo_id] = raw
            if len(positions) == 2:
                break
        initial = tuple(positions.get(i) for i in (1, 2))
        if any(value is None for value in initial):
            missing = [i for i in (1, 2) if i not in positions]
            raise RuntimeError(
                f"Cannot read both gripper positions after 3 attempts; missing IDs={missing}. "
                "Check ESP32 READ support, servo power and bus wiring with bluetooth_position_probe.py"
            )
        config = self.args.config_data
        opened, closed = open_raws(config), close_raws(config)
        if any(a == b for a, b in zip(opened, closed)):
            raise RuntimeError("Invalid gripper open/close range")
        if any(not min(a, b) - 25 <= value <= max(a, b) + 25
               for value, a, b in zip(initial, opened, closed)):
            raise RuntimeError("Measured position outside configured gripper range")
        count = len(self.monitor.ports) - len(self.monitor.ignored_indexes)
        if count < 1:
            raise RuntimeError("No active AnySkin sensor")
        return PositionTracker(opened, closed, initial, self.args.speed,
                               self.args.safe_empty_baseline_margin,
                               self.args.safe_contact_confirm_sec,
                               self.args.safe_contact_confirm_samples,
                               min(count, self.args.safe_contact_min_sensors))

    def _hold_contact_pose(self):
        started = time.monotonic()
        self.motion_timing.pending.clear()
        # An accepted target is not the measured pose: cancel remaining travel.
        firmware_hold = self.args.firmware_hold == "true"
        positions = (self.client.hold() if firmware_hold else
                     self.client.read_contact_positions())
        positions_ready = time.monotonic()
        if any(v is None for v in positions):
            raise RuntimeError("Contact stop could not read both servo positions; motion state unknown")
        if any(not min(a, b) <= v <= max(a, b) for v, a, b in
               zip(positions, self.tracker.open, self.tracker.closed)):
            raise RuntimeError("Contact pose outside configured limits; motion state unknown")
        if not firmware_hold:
            self.client.sync_move(*positions, self.args.hold_speed, self.args.hold_acc)
        self.tracker.acknowledge(positions)
        self.node.get_logger().info(
            f"Contact hold target acknowledged: {positions}; "
            f"request-to-ack={(time.monotonic() - started) * 1000:.1f}ms; "
            f"position-read={(positions_ready - started) * 1000:.1f}ms; "
            f"firmware_hold={firmware_hold}; physical stop not verified")

    def _regrasp_target(self, now, channels, tokens, valid):
        if getattr(self.args, 'slip_regrasp', 'false') != 'true':
            return None
        if not self.tracker.contact:
            self.regrasp.reset()
            self.regrasp_origin = None
            return None
        if self.regrasp_origin is None:
            self.regrasp_origin = tuple(self.tracker.target)
        if not valid or now - self.tracker.received > self.tracker.command_timeout:
            self.regrasp.inhibit('invalid baseline or controller timeout')
            return None
        if not self.regrasp.observe(now, channels, tokens, self.args.safe_empty_baseline_margin):
            return None
        positions = tuple(self.client.read_position(i, timeout=0.1) for i in (1, 2))
        target = bounded_target(positions, self.tracker.target, self.regrasp_origin,
                                self.tracker.open, self.tracker.closed)
        current = time.monotonic()
        with self.condition:
            opening = (self.latest_ratio is None or self.latest_ratio <= 0.003 or
                       self.latest_ratio < self.tracker.hold_input - 0.01)
            stale_command = (not self.web_target_active and
                             (self.latest_received is None or
                              current - self.latest_received > self.tracker.command_timeout))
            cancelled = opening or stale_command or self.manual_command is not None
        stale_sensor = any(current - getattr(stream, 'last_sample_time', 0) > 0.25
                           for stream in self.monitor.streams)
        latest = self.monitor.magnet_strengths()
        expected = empty_baseline_magnet_strengths(
            self.args.config_data, self.tracker.ratio(), len(self.monitor.ports), self.monitor.num_mags)
        latest_rows = [grouped_contact_deltas(v, e) for v, e in zip(latest, expected)]
        flat = [v for row in latest_rows for v in row]
        unsafe_signal = (len(flat) != 6 or not all(math.isfinite(v) for v in flat) or
                         all(max(row) < self.args.safe_empty_baseline_margin for row in latest_rows) or
                         self.regrasp.response_exceeded(flat))
        if target is None or cancelled or stale_sensor or unsafe_signal or self.stop_event.is_set():
            self.regrasp.inhibit('regrasp cancelled: pose, input, or sensor check failed')
            return None
        self.regrasp_before_move = flat
        self.node.get_logger().warn(
            f'Suspected-slip regrasp {self.regrasp.attempts}/3: '
            f'measured={positions}, target={target}; contact/lever hold retained')
        return target

    def _rezero_at_full_open(self, now):
        desired = self.tracker.desired
        if desired is None or now - self.tracker.received > self.tracker.command_timeout:
            return False
        if desired > 0.05:
            self.open_rezero_done = False
        if (desired > 0.003 or self.tracker.ratio() > 0.003 or self.open_rezero_done
                or self.args.open_rezero_samples <= 0 or not self.monitor.enabled):
            return False
        if now < self.open_rezero_check_at:
            return False
        self.open_rezero_check_at = now + 0.5
        def reached():
            positions = [self.client.read_position(i, timeout=0.25) for i in (1, 2)]
            return all(v is not None and abs(v - goal) <= self.args.open_position_tolerance_raw
                       for v, goal in zip(positions, self.tracker.open))
        if not reached():
            return False
        if self.stop_event.wait(max(0.0, self.args.open_rezero_delay_sec)):
            return True
        with self.condition:
            if self.latest_ratio is None or self.latest_ratio > 0.003:
                return False
        if not reached():
            return False
        self.message = "full open reached; rezeroing AnySkin"
        self.node.get_logger().info(self.message)
        self._set_feedback(False)
        # The motion worker owns calibration, so no closing command runs during it.
        self.monitor.calibrate(self.args.open_rezero_samples)
        self.tracker.timers.clear()
        self.tracker.last_tokens.clear()
        self.open_rezero_done = True
        self.message = "AnySkin rezero complete at full open"
        self.node.get_logger().info(self.message)
        return True

    def _worker_loop(self):
        try:
            self.tracker = self._initial_tracker()
            previous_state = None
            last_time = time.monotonic()
            while not self.stop_event.wait(0.002):
                with self.condition:
                    manual = self.manual_command
                    ratio, received = self.latest_ratio, self.latest_received
                if manual:
                    # Calibration is serialized with all motion commands.
                    self.message = "empty-close calibration running"
                    self.web_commands.lock.acquire()
                    self.web_commands._run(manual, self.stop_event.is_set)
                    if "failed:" in self.web_commands.message:
                        raise RuntimeError(self.web_commands.message)
                    self.tracker = self._initial_tracker()
                    with self.condition:
                        self.manual_command = None
                        self.latest_ratio = None
                        self.latest_received = None
                        self.web_target_active = False
                        self.startup_calibration_ready = True
                        self.await_open_after_calibration = True
                    self.message = "Calibration complete; fully open controller lever to enable control"
                    self.node.get_logger().info(self.message)
                    continue
                if not self.startup_calibration_ready or self.await_open_after_calibration:
                    self.message = ("Clear gripper and press Calibrate Empty Close before control"
                                    if not self.startup_calibration_ready else
                                    "Calibration complete; fully open controller lever to enable control")
                    if self.message != previous_state:
                        self.node.get_logger().info(self.message)
                        previous_state = self.message
                    continue
                now = time.monotonic()
                if ratio is not None:
                    self.tracker.command(ratio, now if self.web_target_active else received)
                if self._rezero_at_full_open(now):
                    last_time = time.monotonic()
                    continue
                config = self.args.config_data
                valid = baseline_has_per_magnet_values(config) and baseline_matches_motion(config)
                values = self.monitor.magnet_strengths()
                expected = empty_baseline_magnet_strengths(
                    config, self.tracker.ratio(), len(self.monitor.ports), self.monitor.num_mags)
                channels, tokens = [], []
                for i in range(len(self.monitor.ports)):
                    if i in self.monitor.ignored_indexes:
                        continue
                    stream = self.monitor.streams[i]
                    if (i >= len(values) or now - getattr(stream, "last_sample_time", 0) > 0.25):
                        channels.append([float("nan")])
                    else:
                        channels.append(grouped_contact_deltas(values[i], expected[i]))
                    tokens.append(getattr(stream, "sample_cnt", 0))
                was_contact = self.tracker.contact
                target = self.tracker.tick(now, now - last_time, channels, tokens, valid)
                if self.tracker.contact and not was_contact:
                    # Publish lever feedback immediately, then stop before report work.
                    self._set_feedback(True)
                    self._hold_contact_pose()
                if not self.tracker.contact:
                    self.regrasp.reset()
                    self.regrasp_origin = None
                if (self.tracker.contact != was_contact or
                        (self.tracker.contact and now - getattr(self, "last_sensor_record", 0) >= 0.1)):
                    record = self.String()
                    record.data = json.dumps(dict(
                        component="gripper_sensor", seq=getattr(self, "sensor_record_seq", 0),
                        observed_pc_ns=time.monotonic_ns(), contact=self.tracker.contact,
                        transition=self.tracker.contact != was_contact,
                        strengths=values, baseline=expected, grouped_deltas=channels,
                        margin=self.args.safe_empty_baseline_margin,
                        regrasp_state=self.regrasp.state,
                        regrasp_attempts=self.regrasp.attempts,
                        regrasp_range_low=self.regrasp.range_low,
                        regrasp_range_high=self.regrasp.reference,
                        regrasp_range_excursion=self.regrasp.change_by_sensor,
                        regrasp_direction_by_sensor=self.regrasp.direction_by_sensor,
                        regrasp_response_reference=self.regrasp.response_reference,
                        ports=list(self.monitor.ports)), allow_nan=True)
                    self.timing_publisher.publish(record)
                    self.last_sensor_record = now
                    self.sensor_record_seq = getattr(self, "sensor_record_seq", 0) + 1
                last_time = now
                self.message = self.tracker.state
                if self.feedback_active != self.tracker.contact:
                    self._set_feedback(self.tracker.contact)
                regrasp_move = False
                # Initial hold can block on READ/ACK. Do not feed its older sensor
                # snapshot into regrasp; next loop collects fresh input and samples.
                if target is None and was_contact:
                    extra_target = self._regrasp_target(time.monotonic(), channels, tokens, valid)
                    if extra_target is not None:
                        target = extra_target
                        regrasp_move = True
                if getattr(self.args, 'slip_regrasp', 'false') == 'true':
                    if self.regrasp.state != getattr(self, 'last_regrasp_state', None):
                        self.node.get_logger().info(
                            f'Regrasp: {self.regrasp.state}; '
                            f'change={self.regrasp.change_by_sensor}; attempts={self.regrasp.attempts}/3')
                        self.last_regrasp_state = self.regrasp.state
                if self.message != previous_state:
                    self.node.get_logger().info(
                        f"Gripper state: {self.message}; input={ratio}; "
                        f"target={self.tracker.target}; deltas={channels}"
                    )
                    previous_state = self.message
                if target is not None and not self.stop_event.is_set():
                    if regrasp_move:
                        self.regrasp.note_move(self.regrasp_before_move)
                    self.client.sync_move(*target, 80 if regrasp_move else self.args.speed, self.args.acc)
                    self.tracker.acknowledge(target)
                    try:
                        timing = dict(self.client.last_move_timing)
                        self.motion_timing.command(target, timing['send_start_ns'], getattr(self, 'timing_seq', 0))
                        timing.update(component="gripper", received_ns=int(received * 1e9),
                                      target_raw=list(target), seq=getattr(self, "timing_seq", 0))
                        timing['regrasp'] = regrasp_move
                        self.timing_seq = timing["seq"] + 1
                        message = self.String()
                        message.data = json.dumps(timing)
                        self.timing_publisher.publish(message)
                    except Exception as exc:
                        self.node.get_logger().warn(f"Gripper timing unavailable: {exc}", throttle_duration_sec=5.0)
                with self.condition:
                    new_motion_input = self.latest_ratio != ratio or self.manual_command is not None
                if (target is None and not new_motion_input
                        and time.monotonic() >= self.next_position_poll
                        and not self.stop_event.is_set()):
                    servo_id = self.position_poll_id
                    self.position_poll_id = 3 - servo_id
                    raw = self.client.read_position(servo_id, timeout=0.05)
                    observed_ns = time.monotonic_ns()
                    self.next_position_poll = time.monotonic() + 0.1
                    if raw is not None:
                        event = self.motion_timing.observe(servo_id, raw, observed_ns)
                        if event:
                            message = self.String()
                            message.data = json.dumps(event)
                            self.timing_publisher.publish(message)
        except Exception as exc:
            self.message = (
                f"Gripper motion inhibited: {exc}. Motor stop NOT confirmed. "
                "UR3 body, sensors and recording remain active; inspect gripper before restart."
            )
            self.failure = self.message
            with self.condition:
                self.latest_ratio = None
                self.latest_received = None
                self.manual_command = None
                self.web_target_active = False
            self._set_feedback(False)
            self.node.get_logger().error(self.message)

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=1.0)
        super().close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-startup-calibration', action='store_true')
    parser.add_argument("--gripper-host", default="127.0.0.1")
    parser.add_argument("--firmware-hold", choices=("true", "false"), default="false")
    parser.add_argument("--trace-commands", action="store_true")
    parser.add_argument("--gripper-port", type=int, default=15555)
    parser.add_argument(
        "--gripper-connect-timeout-sec",
        type=float,
        default=0.0,
        help="Exit if the Windows Bluetooth bridge is unavailable for this long; 0 retries forever.",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--anyskin-ports", required=True)
    parser.add_argument("--anyskin-num-mags", type=int, default=5)
    parser.add_argument("--anyskin-baudrate", type=int, default=115200)
    parser.add_argument("--anyskin-warmup-sec", type=float, default=1.0)
    parser.add_argument("--anyskin-startup-timeout-sec", type=float, default=15.0)
    parser.add_argument("--anyskin-stale-timeout-sec", type=float, default=3.0)
    parser.add_argument("--anyskin-reconnect-delay-sec", type=float, default=0.5)
    parser.add_argument("--anyskin-set-control-lines", action="store_true")
    parser.add_argument("--anyskin-filter-alpha", type=float, default=0.35)
    parser.add_argument("--anyskin-filter-release-alpha", type=float, default=0.75)
    parser.add_argument("--anyskin-spike-step-limit", type=float, default=80.0)
    parser.add_argument("--allow-partial-anyskin", action="store_true")
    parser.add_argument("--contact-threshold", type=float, default=150.0)
    parser.add_argument("--contact-thresholds", default="")
    parser.add_argument("--ignore-anyskin-indexes", default="")
    parser.add_argument("--calibration-samples", type=int, default=300)
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    parser.add_argument("--acc", type=int, default=DEFAULT_ACC)
    parser.add_argument("--hold-speed", type=int, default=20)
    parser.add_argument("--hold-acc", type=int, default=50)
    parser.add_argument("--safe-contact-confirm-sec", type=float, default=0.04)
    parser.add_argument("--safe-contact-confirm-samples", type=int, default=3)
    parser.add_argument("--safe-poll-sec", type=float, default=0.01)
    parser.add_argument("--safe-timeout-sec", type=float, default=5.0)
    parser.add_argument("--safe-min-close-sec", type=float, default=0.08)
    parser.add_argument("--safe-empty-baseline-margin", type=float, default=20.0)
    parser.add_argument("--safe-contact-min-channels", type=int, default=1)
    parser.add_argument("--slip-regrasp", choices=("true", "false"), default="false")
    parser.add_argument("--safe-contact-min-sensors", type=int, default=2)
    parser.add_argument("--safe-single-channel-strong-delta", type=float, default=220.0)
    parser.add_argument("--safe-min-contact-ratio", type=float, default=0.18)
    parser.add_argument("--safe-close-steps", type=int, default=160)
    parser.add_argument("--safe-close-step-sec", type=float, default=0.014)
    parser.add_argument("--safe-stop-backoff-raw", type=int, default=0)
    parser.add_argument("--allow-unbaselined-close-safe", action="store_true")
    parser.add_argument("--material-soft-delta", type=float, default=80.0)
    parser.add_argument("--material-hard-delta", type=float, default=180.0)
    parser.add_argument("--material-soft-rise-rate", type=float, default=450.0)
    parser.add_argument("--material-hard-rise-rate", type=float, default=1200.0)
    parser.add_argument("--material-rise-window-sec", type=float, default=0.20)
    parser.add_argument("--empty-close-steps", type=int, default=32)
    parser.add_argument("--empty-close-step-sec", type=float, default=0.035)
    parser.add_argument("--empty-close-point-samples", type=int, default=8)
    parser.add_argument("--empty-close-baseline-percentile", type=float, default=75.0)
    parser.add_argument("--empty-close-cycles", type=int, default=2)
    parser.add_argument("--open-rezero-samples", type=int, default=300)
    parser.add_argument("--open-rezero-delay-sec", type=float, default=0.5)
    parser.add_argument("--open-position-tolerance-raw", type=int, default=25)
    parser.add_argument("--open-settle-timeout-sec", type=float, default=2.5)
    parser.add_argument("--open-stability-sec", type=float, default=0.35)
    parser.add_argument("--open-stability-threshold", type=float, default=8.0)
    parser.add_argument("--print-interval-sec", type=float, default=0.25)
    parser.add_argument("--web-host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--no-web", action="store_true")
    parser.add_argument("--record-anyskin-raw", choices=("true", "false"), default="true")
    parser.add_argument("--anyskin-raw-dir", type=Path,
                        default=Path(__file__).parent / 'anyskin_raw_records')
    parser.add_argument("--ros-command-topic", default="")
    parser.add_argument("--ros-tracking-mode", choices=("discrete", "position"), default="discrete")
    parser.add_argument(
        "--ros-feedback-topic", default="/gello/gripper_contact_hold"
    )
    parser.add_argument(
        "--ros-actual-position-topic", default="/gello/gripper_actual_raw"
    )
    parser.add_argument("--ros-input-min-width", type=float, default=0.012)
    parser.add_argument("--ros-input-max-width", type=float, default=0.13)
    args = parser.parse_args()
    args.config_data = load_config(args.config)

    monitor = AnySkinMonitor(
        parse_ports(args.anyskin_ports),
        args.anyskin_num_mags,
        args.anyskin_baudrate,
        args.contact_threshold,
        parse_float_list(args.contact_thresholds),
        parse_one_based_index_set(args.ignore_anyskin_indexes),
        args.calibration_samples,
        args.anyskin_warmup_sec,
        args.anyskin_startup_timeout_sec,
        args.allow_partial_anyskin,
        False,
        30.0,
        True,
        args.anyskin_stale_timeout_sec,
        args.anyskin_reconnect_delay_sec,
        args.anyskin_set_control_lines,
        args.anyskin_filter_alpha,
        args.anyskin_filter_release_alpha,
        args.anyskin_spike_step_limit,
    )
    client = BluetoothGripperClient(args.gripper_host, args.gripper_port)
    client.trace_commands = args.trace_commands
    server = None
    ros_controller = None
    raw_recorder = None

    try:
        monitor.start()
        if args.record_anyskin_raw == 'true':
            try:
                raw_recorder = AnySkinRawRecorder(monitor, args.anyskin_raw_dir)
                raw_recorder.start()
            except Exception as exc:
                print(f'AnySkin raw recording unavailable (control unchanged): {exc}')
        connect_started_at = time.monotonic()
        while True:
            try:
                client.connect()
                break
            except OSError as exc:
                if (
                    args.gripper_connect_timeout_sec > 0.0
                    and time.monotonic() - connect_started_at
                    >= args.gripper_connect_timeout_sec
                ):
                    raise RuntimeError(
                        "Bluetooth gripper bridge did not become ready within "
                        f"{args.gripper_connect_timeout_sec:.1f}s"
                    ) from exc
                print(
                    "Bluetooth gripper bridge is not ready "
                    f"at {args.gripper_host}:{args.gripper_port}: {exc}. "
                    "Retrying in 2 seconds."
                )
                time.sleep(2.0)
        web_commands = WebGripperCommands(client, monitor, args)
        if args.ros_command_topic:
            def ros_open_action() -> None:
                open_gripper(
                    client,
                    monitor,
                    args.config_data,
                    args.speed,
                    args.acc,
                    args.open_rezero_samples,
                    args.open_rezero_delay_sec,
                    args.open_position_tolerance_raw,
                    args.open_settle_timeout_sec,
                    args.open_stability_sec,
                    args.open_stability_threshold,
                )

            def ros_close_action(cancel_requested: Callable[[], bool]) -> dict:
                return close_safe(
                    client,
                    monitor,
                    args.config_data,
                    args.speed,
                    args.acc,
                    args.safe_contact_confirm_sec,
                    args.safe_poll_sec,
                    args.safe_timeout_sec,
                    args.hold_speed,
                    args.hold_acc,
                    args.print_interval_sec,
                    args.safe_min_close_sec,
                    args.safe_empty_baseline_margin,
                    args.safe_contact_confirm_samples,
                    args.safe_contact_min_channels,
                    args.safe_contact_min_sensors,
                    args.safe_single_channel_strong_delta,
                    args.safe_min_contact_ratio,
                    args.safe_close_steps,
                    args.safe_close_step_sec,
                    args.safe_stop_backoff_raw,
                    args.material_soft_delta,
                    args.material_hard_delta,
                    args.material_soft_rise_rate,
                    args.material_hard_rise_rate,
                    args.material_rise_window_sec,
                    args.allow_unbaselined_close_safe,
                    cancel_requested,
                )

            if args.ros_tracking_mode == "position":
                ros_controller = RosPositionGripperController(client, monitor, args, web_commands)
            else:
                ros_controller = RosDiscreteGripperController(
                    args.ros_command_topic,
                    args.ros_input_min_width,
                    args.ros_input_max_width,
                    ros_open_action,
                    ros_close_action,
                    args.ros_feedback_topic,
                    args.ros_actual_position_topic,
                )
            client.position_callback = ros_controller.publish_actual_position
        if not args.no_web:
            server = start_web_server(
                monitor,
                args.web_host,
                args.web_port,
                ros_controller.submit if ros_controller is not None else web_commands.submit,
                ros_controller.status if isinstance(ros_controller, RosPositionGripperController) else web_commands.status,
            )
        print("Connected. Commands:")
        print("  open [speed] [acc]")
        print("  close [speed] [acc]")
        print("  close_safe [speed] [acc]")
        print("  empty_close_calibrate [speed] [acc]")
        print("  tactile")
        print("  tactile_rezero [samples]")
        print("  read")
        print("  quit")

        while True:
            if ros_controller is not None:
                # A latched gripper fault must not terminate the body or sensor server.
                # The motion worker has exited; callbacks reject all subsequent moves.
                time.sleep(0.25)
                continue
            command_line = input("> ").strip()
            if not command_line:
                continue
            parts = command_line.split()
            command = parts[0].lower()
            speed = int(parts[1]) if len(parts) >= 2 else args.speed
            acc = int(parts[2]) if len(parts) >= 3 else args.acc

            if command in {"quit", "exit"}:
                break
            if command == "open":
                open_gripper(
                    client,
                    monitor,
                    args.config_data,
                    speed,
                    acc,
                    args.open_rezero_samples,
                    args.open_rezero_delay_sec,
                    args.open_position_tolerance_raw,
                    args.open_settle_timeout_sec,
                    args.open_stability_sec,
                    args.open_stability_threshold,
                )
            elif command == "close":
                close_gripper(client, args.config_data, speed, acc)
            elif command == "empty_close_calibrate":
                calibrate_empty_close_baseline(
                    client,
                    monitor,
                    args.config_data,
                    args.config,
                    speed,
                    acc,
                    args.calibration_samples,
                    args.empty_close_steps,
                    args.empty_close_step_sec,
                    args.empty_close_point_samples,
                    args.empty_close_baseline_percentile,
                    args.empty_close_cycles,
                )
            elif command == "close_safe":
                close_safe(
                    client,
                    monitor,
                    args.config_data,
                    speed,
                    acc,
                    args.safe_contact_confirm_sec,
                    args.safe_poll_sec,
                    args.safe_timeout_sec,
                    args.hold_speed,
                    args.hold_acc,
                    args.print_interval_sec,
                    args.safe_min_close_sec,
                    args.safe_empty_baseline_margin,
                    args.safe_contact_confirm_samples,
                    args.safe_contact_min_channels,
                    args.safe_contact_min_sensors,
                    args.safe_single_channel_strong_delta,
                    args.safe_min_contact_ratio,
                    args.safe_close_steps,
                    args.safe_close_step_sec,
                    args.safe_stop_backoff_raw,
                    args.material_soft_delta,
                    args.material_hard_delta,
                    args.material_soft_rise_rate,
                    args.material_hard_rise_rate,
                    args.material_rise_window_sec,
                    args.allow_unbaselined_close_safe,
                )
            elif command == "tactile":
                for line in monitor.status_lines():
                    print(line)
            elif command == "tactile_rezero":
                samples = int(parts[1]) if len(parts) >= 2 else args.calibration_samples
                rezero_anyskin(monitor, samples)
            elif command == "read":
                position1 = client.read_position(1)
                position2 = client.read_position(2)
                print(f"current: id1={position1}, id2={position2}")
            else:
                print("unknown command")
    except KeyboardInterrupt:
        print()
    finally:
        client.position_callback = None
        if ros_controller is not None:
            ros_controller.close()
        if server is not None:
            server.server_close()
        client.close()
        if raw_recorder is not None:
            raw_recorder.close()
        monitor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
