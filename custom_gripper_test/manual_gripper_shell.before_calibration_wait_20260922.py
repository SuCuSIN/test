"""Interactive STS3215 gripper shell that keeps the serial bus open."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
import struct
import threading
import time
from pathlib import Path
from typing import Iterable

from sts3215_bus import (
    MAX_SERVO_ID,
    MIN_SERVO_ID,
    open_bus,
    ping,
    read_position,
    write_position,
)


DEFAULT_CONFIG = Path(__file__).with_name("gripper_motion_ranges.json")
DEFAULT_OPEN_RAWS = [2000, 2000]
DEFAULT_CLOSE_RAWS = [1520, 2480]
DEFAULT_OPEN_SPEED = 220
DEFAULT_OPEN_ACC = 18
DEFAULT_CLOSE_SPEED = 160
DEFAULT_CLOSE_ACC = 12
DEFAULT_ANYSKIN_FILTER_ALPHA = 0.35
DEFAULT_ANYSKIN_FILTER_RELEASE_ALPHA = 0.75
DEFAULT_ANYSKIN_SPIKE_STEP_LIMIT = 80.0


def parse_ids(text: str) -> list[int]:
    ids = [int(item.strip()) for item in text.split(",") if item.strip()]
    if not ids:
        raise argparse.ArgumentTypeError("At least one servo ID is required")
    for servo_id in ids:
        if not MIN_SERVO_ID <= servo_id <= MAX_SERVO_ID:
            raise argparse.ArgumentTypeError(
                f"Servo ID must be {MIN_SERVO_ID}..{MAX_SERVO_ID}: {servo_id}"
            )
    return ids


def parse_ports(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def parse_one_based_index_set(text: str) -> set[int]:
    indexes: set[int] = set()
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        index = int(item)
        if index <= 0:
            raise argparse.ArgumentTypeError("Sensor indexes start at 1")
        indexes.add(index - 1)
    return indexes


def load_config(path: Path) -> dict:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8-sig"))


def save_config(path: Path, config: dict) -> None:
    path.write_text(json.dumps(config, indent=2, sort_keys=True), encoding="utf-8")


def read_positions(ser, ids: Iterable[int]) -> dict[int, int | None]:
    return {servo_id: read_position(ser, servo_id) for servo_id in ids}


def print_positions(label: str, positions: dict[int, int | None]) -> None:
    text = ", ".join(
        f"id{servo_id}={position if position is not None else 'None'}"
        for servo_id, position in positions.items()
    )
    print(f"{label}: {text}")


def parse_speed_acc(parts: list[str], default_speed: int, default_acc: int) -> tuple[int, int]:
    speed = int(parts[0]) if len(parts) >= 1 else default_speed
    acc = int(parts[1]) if len(parts) >= 2 else default_acc
    return speed, acc


def preset_raws(command: str, ids: list[int], config: dict) -> list[int] | None:
    saved_pose = config.get(command)
    if saved_pose and all(str(servo_id) in saved_pose for servo_id in ids):
        return [int(saved_pose[str(servo_id)]) for servo_id in ids]
    if len(ids) == 2 and command == "open":
        return DEFAULT_OPEN_RAWS.copy()
    if len(ids) == 2 and command == "close":
        return DEFAULT_CLOSE_RAWS.copy()
    return None


def send_pair(
    ser,
    ids: list[int],
    raws: list[int],
    speed: int,
    acc: int,
    label: str = "move",
    echo: bool = True,
) -> list[bool]:
    results = []
    for servo_id, raw in zip(ids, raws):
        ok = write_position(ser, servo_id, int(raw), speed, acc)
        results.append(ok)
        if echo:
            print(f"{label} id={servo_id} raw={int(raw)} speed={speed} acc={acc} ok={ok}")
        time.sleep(0.01)
    return results


class AnySkinSerialStream:
    """Direct serial reader for the AnySkin binary burst firmware."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        num_mags: int,
        burst_mode: bool,
        max_samples: int = 1000,
        reconnect_delay_sec: float = 0.5,
        set_control_lines: bool = False,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.num_mags = num_mags
        self.burst_mode = burst_mode
        self.reconnect_delay_sec = reconnect_delay_sec
        self.set_control_lines = set_control_lines
        self.sample_cnt = 0
        self.last_sample_time = 0.0
        self.last_error = ""
        self.connected = False
        self._samples = deque(maxlen=max_samples)
        self._model_samples = deque(maxlen=max_samples)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial = None
        self._serial_module = None

    def start(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for AnySkin serial reading.") from exc

        self._serial_module = serial
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def terminate(self) -> None:
        self._stop.set()
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        self.connected = False

    def pause_streaming(self) -> None:
        self.terminate()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def get_data(self, num_samples: int = 1):
        with self._lock:
            if num_samples <= 0:
                return []
            return list(self._samples)[-num_samples:]

    def raw_samples_after(self, sequence: int):
        """Copy buffered XYZ samples without consuming the control reader's data."""
        with self._lock:
            count = self.sample_cnt
            first = count - len(self._samples) + 1
            rows = [(first + i, list(row)) for i, row in enumerate(self._samples)
                    if first + i > sequence]
        missing = max(0, first - sequence - 1)
        return rows, missing

    def model_data(self, samples=1):
        with self._lock:
            return [list(row) for row in list(self._model_samples)[-samples:]]

    def _record(self, values: list[float], model_xyz=None) -> None:
        now = time.monotonic()
        with self._lock:
            self._samples.append([now, *values])
            self.sample_cnt += 1
            self.last_sample_time = now
            if model_xyz is not None:
                self._model_samples.append([now, *model_xyz])

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._open_serial()
                self.last_error = ""
                if self.burst_mode:
                    self._read_binary_burst_loop()
                else:
                    self._read_ascii_loop()
            except Exception as exc:
                self.last_error = str(exc)
            finally:
                self._close_serial()
            if not self._stop.is_set():
                time.sleep(self.reconnect_delay_sec)

    def _open_serial(self) -> None:
        if self._serial_module is None:
            raise RuntimeError("serial module is not initialized")
        self._serial = self._serial_module.Serial(
            self.port,
            self.baudrate,
            timeout=0.02,
            rtscts=False,
            dsrdtr=False,
        )
        if self.set_control_lines:
            self._serial.dtr = True
            self._serial.rts = True
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass
        self.connected = True

    def _close_serial(self) -> None:
        self.connected = False
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        self._serial = None

    def _read_ascii_loop(self) -> None:
        assert self._serial is not None
        while not self._stop.is_set():
            if self._serial is None:
                return
            line = self._serial.readline()
            if not line:
                continue
            try:
                text = line.decode("ascii", errors="ignore").replace(",", " ")
                values = [float(item) for item in text.split()]
            except ValueError:
                continue
            if len(values) >= self.num_mags * 3:
                self._record(values[: self.num_mags * 3])

    def _read_binary_burst_loop(self) -> None:
        assert self._serial is not None
        msg_floats = 4 * self.num_mags
        msg_length = 4 * msg_floats + 2
        unpack_format = "@{}fcc".format(msg_floats)
        buffer = bytearray()

        while not self._stop.is_set():
            if self._serial is None:
                return
            waiting = getattr(self._serial, "in_waiting", 0)
            chunk = self._serial.read(max(1, waiting))
            if not chunk:
                continue
            buffer.extend(chunk)

            while True:
                end = buffer.find(b"\r\n")
                if end < 0:
                    if len(buffer) > msg_length * 4:
                        del buffer[: -msg_length]
                    break
                packet_end = end + 2
                packet_start = packet_end - msg_length
                if packet_start >= 0:
                    packet = bytes(buffer[packet_start:packet_end])
                    try:
                        unpacked = struct.unpack(unpack_format, packet)
                    except struct.error:
                        del buffer[:packet_end]
                        continue
                    values = [float(value) for value in unpacked[:msg_floats]]
                    xyz_values = []
                    for mag_index in range(self.num_mags):
                        base = mag_index * 4
                        xyz_values.extend(values[base : base + 3])
                    # Keep legacy control unchanged; model input excludes temperature.
                    model_xyz = [values[base + axis] for base in range(0, msg_floats, 4)
                                 for axis in (1, 2, 3)]
                    self._record(xyz_values, model_xyz=model_xyz)
                del buffer[:packet_end]


class AnySkinMonitor:
    """Small contact detector around one or more AnySkin serial sensors."""

    def __init__(
        self,
        ports: list[str],
        num_mags: int,
        baudrate: int,
        threshold: float,
        thresholds: list[float],
        ignored_indexes: set[int],
        calibration_samples: int,
        warmup_sec: float,
        startup_timeout_sec: float,
        allow_partial: bool,
        show_viz: bool,
        viz_rate_hz: float,
        burst_mode: bool,
        stale_timeout_sec: float,
        reconnect_delay_sec: float,
        set_control_lines: bool,
        filter_alpha: float = DEFAULT_ANYSKIN_FILTER_ALPHA,
        filter_release_alpha: float = DEFAULT_ANYSKIN_FILTER_RELEASE_ALPHA,
        spike_step_limit: float = DEFAULT_ANYSKIN_SPIKE_STEP_LIMIT,
    ) -> None:
        self.ports = ports
        self.num_mags = num_mags
        self.baudrate = baudrate
        self.threshold = threshold
        self.thresholds = thresholds
        self.ignored_indexes = ignored_indexes
        self.calibration_samples = calibration_samples
        self.warmup_sec = warmup_sec
        self.startup_timeout_sec = startup_timeout_sec
        self.allow_partial = allow_partial
        self.show_viz = show_viz
        self.viz_rate_hz = viz_rate_hz
        self.burst_mode = burst_mode
        self.stale_timeout_sec = stale_timeout_sec
        self.reconnect_delay_sec = reconnect_delay_sec
        self.set_control_lines = set_control_lines
        self.filter_alpha = max(0.0, min(1.0, filter_alpha))
        self.filter_release_alpha = max(0.0, min(1.0, filter_release_alpha))
        self.spike_step_limit = max(0.0, spike_step_limit)
        self.streams = []
        self.baselines = []
        self._filtered_magnet_strengths: list[list[float]] = []
        self.last_calibration_samples = 0
        self.last_calibration_time = 0.0
        self.raw_calibration_monotonic = 0.0
        self._np = None
        self._sample_lock = threading.Lock()
        self._viz_stop = threading.Event()
        self._viz_thread: threading.Thread | None = None

    @property
    def enabled(self) -> bool:
        return bool(self.ports)

    def threshold_for_index(self, index: int) -> float:
        if index < len(self.thresholds):
            return self.thresholds[index]
        return self.threshold

    def start(self) -> None:
        if not self.enabled:
            return

        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError(
                "numpy is required for AnySkin contact detection."
            ) from exc

        self._np = np
        candidates = []
        for port in self.ports:
            stream = AnySkinSerialStream(
                port,
                self.baudrate,
                self.num_mags,
                self.burst_mode,
                reconnect_delay_sec=self.reconnect_delay_sec,
                set_control_lines=self.set_control_lines,
            )
            stream.start()
            candidates.append((port, stream))

        print(f"AnySkin starting on {', '.join(self.ports)}...")
        time.sleep(self.warmup_sec)
        deadline = time.monotonic() + self.startup_timeout_sec
        while time.monotonic() < deadline:
            if all(getattr(stream, "sample_cnt", 0) > 0 for _, stream in candidates):
                break
            time.sleep(0.05)

        good = [
            (port, stream)
            for port, stream in candidates
            if getattr(stream, "sample_cnt", 0) > 0 and not getattr(stream, "last_error", "")
        ]
        bad_ports = [
            port
            for port, stream in candidates
            if getattr(stream, "sample_cnt", 0) <= 0 or getattr(stream, "last_error", "")
        ]
        if bad_ports and not self.allow_partial:
            for _, stream in candidates:
                self._stop_stream(stream)
            raise RuntimeError(
                "No AnySkin samples received from "
                f"{', '.join(bad_ports)}. Check ports or add --allow-partial-anyskin."
            )
        for _, stream in candidates:
            if getattr(stream, "sample_cnt", 0) <= 0 or getattr(stream, "last_error", ""):
                self._stop_stream(stream)

        self.ports = [port for port, _ in good]
        self.streams = [stream for _, stream in good]
        if not self.streams:
            raise RuntimeError("No AnySkin sensor produced samples.")
        if bad_ports:
            print(f"WARNING: ignoring non-streaming AnySkin ports: {', '.join(bad_ports)}")
        self.calibrate(self.calibration_samples)
        if self.show_viz:
            self.start_visualizer()

    def close(self) -> None:
        self.stop_visualizer()
        for stream in self.streams:
            self._stop_stream(stream)
        self.streams = []

    def _stop_stream(self, stream) -> None:
        try:
            if getattr(stream, "is_alive", lambda: False)():
                stream.terminate()
        except Exception:
            pass
        try:
            stream.join(timeout=0.5)
        except Exception:
            pass
        try:
            stream.pause_streaming()
        except Exception:
            pass
        try:
            stream.join(timeout=0.5)
        except Exception:
            pass

    def _sample(self, stream):
        if getattr(stream, "is_alive", lambda: False)() is False:
            return None
        if getattr(stream, "last_error", ""):
            return None
        last_sample_time = getattr(stream, "last_sample_time", 0.0)
        if last_sample_time <= 0.0:
            return None
        if time.monotonic() - last_sample_time > self.stale_timeout_sec:
            return None
        if getattr(stream, "sample_cnt", 0) <= 0:
            return None
        samples = stream.get_data(num_samples=1)
        if not samples:
            return None
        return self._np.asarray(samples[-1][1:], dtype=float)

    def magnet_strengths(self) -> list[list[float]]:
        """Return per-magnetometer delta magnitudes for each AnySkin sensor."""
        if not self.enabled or not self.baselines:
            return []
        strengths = []
        with self._sample_lock:
            for stream, baseline in zip(self.streams, self.baselines):
                sample = self._sample(stream)
                if sample is None:
                    strengths.append([float("nan")] * self.num_mags)
                    continue
                delta = sample - baseline
                if delta.size % 3 == 0:
                    per_mag = self._np.linalg.norm(delta.reshape((-1, 3)), axis=1)
                    strengths.append([float(value) for value in per_mag])
                else:
                    strengths.append([float(self._np.linalg.norm(delta))])
            strengths = self._filter_magnet_strengths(strengths)
        return strengths

    def raw_calibration_snapshot(self):
        """Read-only metadata; does not advance contact filters or read serial."""
        with self._sample_lock:
            return {
                'effective_pc_monotonic_s': self.raw_calibration_monotonic,
                'calibration_wall_time_s': self.last_calibration_time,
                'samples': self.last_calibration_samples,
                'ports': list(self.ports),
                'baseline_xyz_flat': [list(map(float, row)) for row in self.baselines],
            }

    def _filter_magnet_strengths(self, raw_strengths: list[list[float]]) -> list[list[float]]:
        """Clamp one-frame AnySkin spikes while still allowing sustained contact."""
        if self.filter_alpha >= 1.0 and self.spike_step_limit <= 0.0:
            return raw_strengths
        shape_changed = len(raw_strengths) != len(self._filtered_magnet_strengths) or any(
            len(raw_row) != len(filtered_row)
            for raw_row, filtered_row in zip(raw_strengths, self._filtered_magnet_strengths)
        )
        if shape_changed:
            self._filtered_magnet_strengths = [list(row) for row in raw_strengths]
            return [list(row) for row in self._filtered_magnet_strengths]

        filtered_strengths: list[list[float]] = []
        for raw_row, previous_row in zip(raw_strengths, self._filtered_magnet_strengths):
            filtered_row: list[float] = []
            for raw_value, previous_value in zip(raw_row, previous_row):
                if math.isnan(raw_value):
                    filtered_value = float("nan")
                elif math.isnan(previous_value):
                    filtered_value = raw_value
                else:
                    target_value = raw_value
                    if raw_value > previous_value and self.spike_step_limit > 0.0:
                        target_value = min(raw_value, previous_value + self.spike_step_limit)
                    alpha = (
                        self.filter_alpha
                        if target_value >= previous_value
                        else self.filter_release_alpha
                    )
                    filtered_value = previous_value + (target_value - previous_value) * alpha
                    if raw_value < 1.0 and filtered_value < 1.0:
                        filtered_value = 0.0
                filtered_row.append(filtered_value)
            filtered_strengths.append(filtered_row)
        self._filtered_magnet_strengths = filtered_strengths
        return [list(row) for row in filtered_strengths]

    def calibrate(self, samples: int | None = None) -> None:
        if not self.enabled:
            print("AnySkin is disabled. Start with --anyskin-ports <port1,port2>.")
            return
        samples = samples or self.calibration_samples
        baselines = []
        model_baselines = []
        for sensor_index, stream in enumerate(self.streams, start=1):
            start_count = getattr(stream, "sample_cnt", 0)
            deadline = time.monotonic() + max(3.0, samples * 0.05)
            while getattr(stream, "sample_cnt", 0) < start_count + samples and time.monotonic() < deadline:
                time.sleep(0.01)
            readings = self._np.asarray(stream.get_data(num_samples=samples), dtype=float)
            if readings.ndim != 2 or readings.shape[0] < max(1, min(samples, 3)):
                raise RuntimeError(f"No AnySkin samples received from sensor {sensor_index}")
            readings = readings[:, 1:]
            if readings.size == 0:
                raise RuntimeError(f"No AnySkin samples received from sensor {sensor_index}")
            baselines.append(self._np.median(readings, axis=0))
            model_rows = stream.model_data(samples) if hasattr(stream, 'model_data') else []
            model_baselines.append(
                self._np.median(self._np.asarray(model_rows)[:, 1:], axis=0).tolist()
                if len(model_rows) >= samples else None)
        with self._sample_lock:
            self.baselines = baselines
            self.model_baselines = model_baselines
            self._filtered_magnet_strengths = []
            self.last_calibration_samples = samples
            self.last_calibration_time = time.time()
            self.raw_calibration_monotonic = time.monotonic()
        sensor_text = ", ".join(
            f"s{index + 1}={port}" for index, port in enumerate(self.ports)
        )
        print(
            f"AnySkin calibrated separately with {samples} samples "
            f"({sensor_text}). Keep sensors unloaded while calibrating."
        )

    def slip_model_snapshot(self) -> dict:
        with self._sample_lock:
            baselines = getattr(self, 'model_baselines', [])
            epoch = self.raw_calibration_monotonic
        now = time.monotonic()
        sensors = []
        for index, stream in enumerate(self.streams):
            rows = stream.model_data() if hasattr(stream, 'model_data') else []
            row = rows[-1] if rows else None
            sensors.append(dict(
                xyz=row[1:] if row else None,
                received_pc_monotonic_s=row[0] if row else None,
                age_s=now - row[0] if row else None,
                baseline=baselines[index] if index < len(baselines) else None))
        return dict(schema='anyskin_txyz_model_v1', calibration_epoch=epoch, sensors=sensors)

    def calibration_snapshot(self) -> dict:
        """Return the most recent per-sensor, per-magnet calibration baseline."""
        with self._sample_lock:
            baselines = list(self.baselines)
            samples = self.last_calibration_samples
            calibrated_at = self.last_calibration_time

        sensors = []
        for sensor_index, baseline in enumerate(baselines):
            flat_values = [float(value) for value in baseline.tolist()]
            xyz_values = []
            if len(flat_values) >= self.num_mags * 3:
                for mag_index in range(self.num_mags):
                    base = mag_index * 3
                    xyz_values.append(flat_values[base : base + 3])
            sensors.append(
                {
                    "index": sensor_index + 1,
                    "port": self.ports[sensor_index] if sensor_index < len(self.ports) else "",
                    "num_mags": self.num_mags,
                    "xyz_baseline": xyz_values,
                    "flat_baseline": flat_values,
                }
            )

        return {
            "samples": samples,
            "calibrated_at_unix": calibrated_at,
            "sensor_count": len(sensors),
            "sensors": sensors,
        }

    def strengths(self) -> list[float]:
        strengths = []
        for values in self.magnet_strengths():
            finite_values = [value for value in values if not math.isnan(value)]
            strengths.append(max(finite_values) if finite_values else float("nan"))
        return strengths

    def contact(self) -> tuple[bool, list[float]]:
        strengths = self.strengths()
        return any(
            index not in self.ignored_indexes
            and not math.isnan(value)
            and value >= self.threshold_for_index(index)
            for index, value in enumerate(strengths)
        ), strengths

    def status_lines(self) -> list[str]:
        if not self.enabled:
            return ["AnySkin is disabled."]
        strengths = self.strengths()
        now = time.monotonic()
        lines = []
        for index, (port, stream) in enumerate(zip(self.ports, self.streams)):
            strength = strengths[index] if index < len(strengths) else float("nan")
            threshold = self.threshold_for_index(index)
            ignored = index in self.ignored_indexes
            last_time = getattr(stream, "last_sample_time", 0.0)
            age_text = "never" if last_time <= 0.0 else f"{now - last_time:.2f}s"
            error = getattr(stream, "last_error", "")
            if ignored:
                state = "ignored"
            elif math.isnan(strength):
                state = "lost"
            else:
                state = "contact" if strength >= threshold else "clear"
            lines.append(
                f"s{index + 1} {port}: {state}, strength="
                f"{'lost' if math.isnan(strength) else f'{strength:.2f}'}, "
                f"threshold={threshold:.2f}, "
                f"samples={getattr(stream, 'sample_cnt', 0)}, age={age_text}, "
                f"connected={getattr(stream, 'connected', False)}, "
                f"alive={getattr(stream, 'is_alive', lambda: False)()}, error={error or 'none'}"
            )
        return lines

    def start_visualizer(self) -> None:
        if self._viz_thread is not None:
            return
        self._viz_thread = threading.Thread(target=self._visualizer_loop, daemon=True)
        self._viz_thread.start()
        print("AnySkin visualizer window requested.")

    def stop_visualizer(self) -> None:
        self._viz_stop.set()
        if self._viz_thread is not None:
            self._viz_thread.join(timeout=1.0)
            self._viz_thread = None

    def _visualizer_loop(self) -> None:
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "80,80")
        try:
            import pygame
        except ImportError:
            print("WARNING: pygame is not installed; AnySkin visualizer disabled.")
            return

        pygame.init()
        width, height = 760, 420
        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("AnySkin contact monitor")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont(None, 28)
        small_font = pygame.font.SysFont(None, 22)
        panel_count = max(len(self.ports), 1)

        while not self._viz_stop.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._viz_stop.set()

            strengths = self.strengths()
            screen.fill((24, 26, 30))
            title = font.render("AnySkin contact monitor", True, (235, 235, 235))
            screen.blit(title, (24, 20))
            threshold_text = small_font.render(
                "thresholds: "
                + ", ".join(f"{self.threshold_for_index(i):.1f}" for i in range(panel_count)),
                True,
                (190, 196, 205),
            )
            screen.blit(threshold_text, (24, 52))

            panel_width = (width - 72) // panel_count
            for index in range(panel_count):
                x = 24 + index * panel_width
                y = 92
                value = strengths[index] if index < len(strengths) else float("nan")
                threshold = self.threshold_for_index(index)
                ignored = index in self.ignored_indexes
                is_lost = math.isnan(value)
                in_contact = not ignored and not is_lost and value >= threshold
                fill = (
                    (54, 54, 61)
                    if is_lost or ignored
                    else ((63, 26, 28) if in_contact else (28, 43, 48))
                )
                outline = (
                    (120, 126, 138)
                    if is_lost or ignored
                    else ((231, 89, 79) if in_contact else (82, 183, 156))
                )
                pygame.draw.rect(screen, fill, (x, y, panel_width - 16, 270), border_radius=8)
                pygame.draw.rect(screen, outline, (x, y, panel_width - 16, 270), 3, border_radius=8)

                label = f"AnySkin {index + 1}"
                if index < len(self.ports):
                    label += f"  {self.ports[index]}"
                screen.blit(font.render(label, True, (235, 235, 235)), (x + 18, y + 18))

                value_text = "lost" if is_lost else f"{value:.2f}"
                screen.blit(
                    font.render(f"strength {value_text}", True, (235, 235, 235)),
                    (x + 18, y + 58),
                )

                bar_x, bar_y = x + 18, y + 110
                bar_width, bar_height = panel_width - 52, 34
                pygame.draw.rect(screen, (52, 56, 64), (bar_x, bar_y, bar_width, bar_height), border_radius=5)
                if not math.isnan(value):
                    ratio = min(1.0, max(0.0, value / max(threshold, 1.0)))
                    color = (231, 89, 79) if in_contact else (82, 183, 156)
                    pygame.draw.rect(
                        screen,
                        color,
                        (bar_x, bar_y, int(bar_width * ratio), bar_height),
                        border_radius=5,
                    )

                status = "ignored" if ignored else ("lost" if is_lost else ("CONTACT" if in_contact else "clear"))
                status_color = (
                    (205, 210, 220)
                    if is_lost or ignored
                    else ((255, 170, 160) if in_contact else (190, 230, 218))
                )
                screen.blit(font.render(status, True, status_color), (x + 18, y + 166))

            pygame.display.flip()
            clock.tick(max(self.viz_rate_hz, 1.0))

        pygame.quit()


def format_strengths(strengths: list[float]) -> str:
    if not strengths:
        return "none"
    return ", ".join(
        f"s{i + 1}={'lost' if math.isnan(value) else f'{value:.2f}'}"
        for i, value in enumerate(strengths)
    )


def max_finite(values: list[float]) -> float:
    finite_values = [value for value in values if not math.isnan(value)]
    return max(finite_values) if finite_values else float("nan")


def format_magnet_strengths(magnet_strengths: list[list[float]]) -> str:
    if not magnet_strengths:
        return "none"
    parts = []
    for sensor_index, values in enumerate(magnet_strengths, start=1):
        value_text = ", ".join(
            f"M{mag_index + 1}={'lost' if math.isnan(value) else f'{value:.1f}'}"
            for mag_index, value in enumerate(values)
        )
        parts.append(f"s{sensor_index}[{value_text}]")
    return "; ".join(parts)


def sensor_max_strengths(magnet_strengths: list[list[float]]) -> list[float]:
    return [max_finite(values) for values in magnet_strengths]


def empty_baseline_strengths(config: dict, ratio: float, sensor_count: int) -> list[float]:
    baseline = config.get("empty_close_baseline")
    num_mags = int(baseline.get("num_mags", 5)) if baseline else 5
    return sensor_max_strengths(
        empty_baseline_magnet_strengths(config, ratio, sensor_count, num_mags)
    )


def empty_baseline_magnet_strengths(
    config: dict,
    ratio: float,
    sensor_count: int,
    num_mags: int,
) -> list[list[float]]:
    baseline = config.get("empty_close_baseline")
    if not baseline:
        return [[0.0] * num_mags for _ in range(sensor_count)]
    config_open = preset_raws("open", [1, 2], config)
    config_close = preset_raws("close", [1, 2], config)
    baseline_open = baseline.get("open")
    baseline_close = baseline.get("close")
    if (
        config_open is not None
        and config_close is not None
        and isinstance(baseline_open, list)
        and isinstance(baseline_close, list)
        and (
            [int(value) for value in baseline_open[:2]] != config_open[:2]
            or [int(value) for value in baseline_close[:2]] != config_close[:2]
        )
    ):
        return [[0.0] * num_mags for _ in range(sensor_count)]
    points = baseline.get("points", [])
    if not points:
        return [[0.0] * num_mags for _ in range(sensor_count)]

    num_mags = max(1, int(baseline.get("num_mags", num_mags)))

    def read_strength(values: list, index: int, default: float = 0.0) -> float:
        if index >= len(values) or values[index] is None:
            return default
        return float(values[index])

    def point_matrix(point: dict) -> list[list[float]]:
        matrix = point.get("magnet_strengths")
        if isinstance(matrix, list):
            out = []
            for sensor_index in range(sensor_count):
                row = matrix[sensor_index] if sensor_index < len(matrix) else []
                if isinstance(row, list) and row:
                    out.append(
                        [
                            read_strength(row, mag_index)
                            for mag_index in range(num_mags)
                        ]
                    )
                else:
                    sensor_strengths = point.get("strengths", [])
                    fallback = read_strength(sensor_strengths, sensor_index)
                    out.append([fallback] * num_mags)
            return out

        sensor_strengths = point.get("strengths", [])
        return [
            [read_strength(sensor_strengths, sensor_index)] * num_mags
            for sensor_index in range(sensor_count)
        ]

    ratio = max(0.0, min(1.0, ratio))
    previous = points[0]
    for current in points:
        current_ratio = float(current.get("ratio", 0.0))
        if ratio <= current_ratio:
            previous_ratio = float(previous.get("ratio", 0.0))
            span = max(current_ratio - previous_ratio, 1e-9)
            mix = max(0.0, min(1.0, (ratio - previous_ratio) / span))
            previous_matrix = point_matrix(previous)
            current_matrix = point_matrix(current)
            return [
                [
                    previous_matrix[sensor_index][mag_index]
                    + (
                        current_matrix[sensor_index][mag_index]
                        - previous_matrix[sensor_index][mag_index]
                    )
                    * mix
                    for mag_index in range(num_mags)
                ]
                for sensor_index in range(sensor_count)
            ]
        previous = current

    return point_matrix(points[-1])


def backoff_toward_open(
    stop_raws: list[int],
    open_raws: list[int],
    backoff_raw: int,
) -> list[int]:
    if backoff_raw <= 0:
        return stop_raws
    backed_off = []
    for stop_raw, open_raw in zip(stop_raws, open_raws):
        if stop_raw < open_raw:
            raw = min(open_raw, stop_raw + backoff_raw)
        elif stop_raw > open_raw:
            raw = max(open_raw, stop_raw - backoff_raw)
        else:
            raw = stop_raw
        backed_off.append(max(0, min(4095, int(raw))))
    return backed_off


def close_ratio_from_raws(
    raws: list[int] | tuple[int, ...],
    open_raws: list[int] | tuple[int, ...],
    close_raws: list[int] | tuple[int, ...],
) -> float:
    ratios = []
    for raw, open_raw, close_raw in zip(raws, open_raws, close_raws):
        span = close_raw - open_raw
        if span == 0:
            continue
        ratios.append((raw - open_raw) / span)
    if not ratios:
        return 0.0
    return max(0.0, min(1.0, sum(ratios) / len(ratios)))


def calibrate_empty_close_baseline(
    ser,
    ids: list[int],
    config: dict,
    config_path: Path,
    tactile: AnySkinMonitor,
    speed: int,
    acc: int,
    step_raw: int,
    step_interval: float,
) -> None:
    if not tactile.enabled:
        print("AnySkin is disabled. Start with --anyskin-ports <port1,port2>.")
        return

    open_raws = preset_raws("open", ids, config)
    close_raws = preset_raws("close", ids, config)
    if open_raws is None or close_raws is None:
        print("Need saved open and close poses. Run save_open and save_close first.")
        return

    steps = max(1, math.ceil(max(abs(target - start) for start, target in zip(open_raws, close_raws)) / max(step_raw, 1)))
    print("Calibrating empty close baseline. Keep the gripper empty and sensors unloaded.")
    send_pair(ser, ids, open_raws, speed, acc, label="baseline_open", echo=False)
    time.sleep(0.5)
    tactile.calibrate(max(80, tactile.calibration_samples))

    points = []
    for step_index in range(steps + 1):
        ratio = step_index / steps
        target_raws = [
            int(round(start + (target - start) * ratio))
            for start, target in zip(open_raws, close_raws)
        ]
        send_pair(ser, ids, target_raws, speed, acc, label="baseline_close", echo=False)
        time.sleep(max(step_interval, 0.02))
        magnet_strengths = tactile.magnet_strengths()
        strengths = sensor_max_strengths(magnet_strengths)
        points.append(
            {
                "ratio": ratio,
                "raws": target_raws,
                "magnet_strengths": [
                    [
                        None if math.isnan(value) else float(value)
                        for value in sensor_values
                    ]
                    for sensor_values in magnet_strengths
                ],
                "strengths": [
                    None if math.isnan(value) else float(value)
                    for value in strengths
                ],
            }
        )

    config["empty_close_baseline"] = {
        "ids": ids,
        "open": open_raws,
        "close": close_raws,
        "sensor_ports": tactile.ports,
        "num_mags": tactile.num_mags,
        "calibration_baselines": tactile.calibration_snapshot(),
        "points": points,
    }
    save_config(config_path, config)
    send_pair(ser, ids, open_raws, speed, acc, label="baseline_open", echo=False)
    print(f"Saved empty close baseline with {len(points)} points to {config_path}.")


def close_until_contact(
    ser,
    ids: list[int],
    config: dict,
    tactile: AnySkinMonitor,
    speed: int,
    acc: int,
    step_raw: int,
    step_interval: float,
    stop_backoff_raw: int,
    hold_speed: int,
    hold_acc: int,
    hold_repeats: int,
    contact_confirm_sec: float,
    baseline_margin: float,
    contact_confirm_samples: int,
) -> None:
    if not tactile.enabled:
        print("AnySkin is disabled. Start with --anyskin-ports <port1,port2>.")
        return
    if "close" not in config or "open" not in config:
        print("Need saved open and close poses. Run save_open and save_close first.")
        return

    start_positions = read_positions(ser, ids)
    if any(position is None for position in start_positions.values()):
        print_positions("cannot close_safe, missing current position", start_positions)
        return

    start_raws = [int(start_positions[servo_id]) for servo_id in ids]
    close_raws = [int(config["close"][str(servo_id)]) for servo_id in ids]
    open_raws = [int(config["open"][str(servo_id)]) for servo_id in ids]
    steps = max(1, math.ceil(max(abs(target - start) for start, target in zip(start_raws, close_raws)) / max(step_raw, 1)))
    contact_confirm_sec = max(contact_confirm_sec, 0.0)

    print(
        "close_safe started. "
        f"threshold={tactile.threshold:.2f}, confirm={contact_confirm_sec:.3f}s, "
        f"empty_margin={baseline_margin:.2f}, "
        f"steps={steps}, speed={speed}, acc={acc}"
    )

    contact_confirm_samples = max(1, contact_confirm_samples)
    contact_started_at_by_channel: list[list[float | None]] = []
    contact_counts_by_channel: list[list[int]] = []

    def confirmed_contact(ratio: float) -> tuple[bool, list[float], list[float]]:
        nonlocal contact_started_at_by_channel, contact_counts_by_channel
        magnet_strengths = tactile.magnet_strengths()
        strengths = sensor_max_strengths(magnet_strengths)
        expected = empty_baseline_magnet_strengths(
            config,
            ratio,
            len(magnet_strengths),
            tactile.num_mags,
        )
        now = time.monotonic()
        expected_shape = [len(values) for values in magnet_strengths]
        if [len(values) for values in contact_started_at_by_channel] != expected_shape:
            contact_started_at_by_channel = [
                [None] * len(values)
                for values in magnet_strengths
            ]
            contact_counts_by_channel = [
                [0] * len(values)
                for values in magnet_strengths
            ]

        deltas = []
        confirmed = False
        for sensor_index, values in enumerate(magnet_strengths):
            expected_values = expected[sensor_index] if sensor_index < len(expected) else []
            sensor_deltas = []
            for mag_index, value in enumerate(values):
                baseline = expected_values[mag_index] if mag_index < len(expected_values) else 0.0
                delta = float("nan") if math.isnan(value) else value - baseline
                sensor_deltas.append(delta)
                candidate = (
                    sensor_index not in tactile.ignored_indexes
                    and not math.isnan(delta)
                    and delta >= baseline_margin
                )
                if candidate:
                    if contact_started_at_by_channel[sensor_index][mag_index] is None:
                        contact_started_at_by_channel[sensor_index][mag_index] = now
                        contact_counts_by_channel[sensor_index][mag_index] = 0
                    contact_counts_by_channel[sensor_index][mag_index] += 1
                    if (
                        now - contact_started_at_by_channel[sensor_index][mag_index] >= contact_confirm_sec
                        and contact_counts_by_channel[sensor_index][mag_index] >= contact_confirm_samples
                    ):
                        confirmed = True
                else:
                    contact_started_at_by_channel[sensor_index][mag_index] = None
                    contact_counts_by_channel[sensor_index][mag_index] = 0
            deltas.append(max_finite(sensor_deltas))
        return confirmed, strengths, deltas

    initial_deadline = time.monotonic() + contact_confirm_sec
    initial_ratio = close_ratio_from_raws(start_raws, open_raws, close_raws)
    while True:
        is_contact, strengths, _ = confirmed_contact(initial_ratio)
        if is_contact or time.monotonic() >= initial_deadline:
            break
        time.sleep(0.005)
    if is_contact:
        print(f"AnySkin already in confirmed contact before closing: {format_strengths(strengths)}")
        return

    def hold_current_position(
        fallback_raws: list[int],
        strengths: list[float],
    ) -> None:
        stop_positions = read_positions(ser, ids)
        stop_raws = [
            int(stop_positions[servo_id])
            if stop_positions[servo_id] is not None
            else fallback_raws[index]
            for index, servo_id in enumerate(ids)
        ]
        hold_raws = backoff_toward_open(stop_raws, open_raws, stop_backoff_raw)
        for _ in range(max(1, hold_repeats)):
            send_pair(ser, ids, hold_raws, hold_speed, hold_acc, label="hold", echo=False)
            time.sleep(0.01)
        print(
            "AnySkin contact detected; gripper stopped and holding current pose. "
            f"raw={hold_raws}, strength={format_strengths(strengths)}"
        )

    last_commanded = start_raws
    for step_index in range(1, steps + 1):
        previous_ratio = close_ratio_from_raws(last_commanded, open_raws, close_raws)
        is_contact, strengths, _ = confirmed_contact(previous_ratio)
        if is_contact:
            hold_current_position(last_commanded, strengths)
            return

        ratio = step_index / steps
        target_raws = [
            int(round(start + (target - start) * ratio))
            for start, target in zip(start_raws, close_raws)
        ]
        send_pair(ser, ids, target_raws, speed, acc, label="close_safe", echo=False)
        last_commanded = target_raws
        baseline_ratio = close_ratio_from_raws(target_raws, open_raws, close_raws)

        deadline = time.monotonic() + step_interval
        while time.monotonic() < deadline:
            is_contact, strengths, _ = confirmed_contact(baseline_ratio)
            if is_contact:
                hold_current_position(last_commanded, strengths)
                return
            time.sleep(0.005)

    final_deadline = time.monotonic() + contact_confirm_sec
    while True:
        is_contact, strengths, _ = confirmed_contact(1.0)
        if is_contact or time.monotonic() >= final_deadline:
            break
        time.sleep(0.005)
    if is_contact:
        print(
            "AnySkin contact detected at final close pose. "
            f"strength={format_strengths(strengths)}"
        )
    else:
        print("close_safe reached saved close pose without AnySkin contact.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--sensor-only", action="store_true", help="Run only the AnySkin monitor without opening the gripper servo bus.")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", type=parse_ids, default=[1, 2], help="Comma-separated servo IDs")
    parser.add_argument("--speed", type=int, default=250)
    parser.add_argument("--acc", type=int, default=20)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--anyskin-ports", default="", help="Comma-separated AnySkin serial ports")
    parser.add_argument("--anyskin-num-mags", type=int, default=5)
    parser.add_argument("--anyskin-baudrate", type=int, default=115200)
    parser.add_argument("--anyskin-warmup-sec", type=float, default=1.0)
    parser.add_argument("--anyskin-startup-timeout-sec", type=float, default=8.0)
    parser.add_argument("--allow-partial-anyskin", action="store_true")
    parser.add_argument("--show-anyskin-viz", action="store_true")
    parser.add_argument("--anyskin-viz-rate-hz", type=float, default=30.0)
    parser.add_argument("--anyskin-ascii", action="store_true", help="Use if the QT Py runs 5X_burst_stream instead of 5X_binary_burst_stream.")
    parser.add_argument("--anyskin-stale-timeout-sec", type=float, default=1.0)
    parser.add_argument("--anyskin-reconnect-delay-sec", type=float, default=0.5)
    parser.add_argument("--anyskin-set-control-lines", action="store_true")
    parser.add_argument("--anyskin-filter-alpha", type=float, default=DEFAULT_ANYSKIN_FILTER_ALPHA)
    parser.add_argument("--anyskin-filter-release-alpha", type=float, default=DEFAULT_ANYSKIN_FILTER_RELEASE_ALPHA)
    parser.add_argument("--anyskin-spike-step-limit", type=float, default=DEFAULT_ANYSKIN_SPIKE_STEP_LIMIT)
    parser.add_argument("--contact-threshold", type=float, default=110.0)
    parser.add_argument("--contact-thresholds", default="", help="Comma-separated per-sensor thresholds, for example 110,5000.")
    parser.add_argument("--ignore-anyskin-indexes", default="", help="Comma-separated 1-based AnySkin indexes to ignore for contact detection.")
    parser.add_argument("--contact-calibration-samples", type=int, default=20)
    parser.add_argument("--rezero-after-open", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--open-rezero-samples", type=int, default=300)
    parser.add_argument("--safe-close-step-raw", type=int, default=2)
    parser.add_argument("--safe-close-interval-sec", type=float, default=0.02)
    parser.add_argument("--safe-stop-backoff-raw", type=int, default=4)
    parser.add_argument("--safe-hold-speed", type=int, default=35)
    parser.add_argument("--safe-hold-acc", type=int, default=2)
    parser.add_argument("--safe-hold-repeats", type=int, default=3)
    parser.add_argument("--safe-contact-confirm-sec", type=float, default=0.12)
    parser.add_argument("--safe-contact-confirm-samples", type=int, default=5)
    parser.add_argument("--safe-empty-baseline-margin", type=float, default=20.0)
    args = parser.parse_args()
    contact_thresholds = parse_float_list(args.contact_thresholds)
    ignored_anyskin_indexes = parse_one_based_index_set(args.ignore_anyskin_indexes)

    config = load_config(args.config)
    tactile = AnySkinMonitor(
        parse_ports(args.anyskin_ports),
        args.anyskin_num_mags,
        args.anyskin_baudrate,
        args.contact_threshold,
        contact_thresholds,
        ignored_anyskin_indexes,
        args.contact_calibration_samples,
        args.anyskin_warmup_sec,
        args.anyskin_startup_timeout_sec,
        args.allow_partial_anyskin,
        args.show_anyskin_viz,
        args.anyskin_viz_rate_hz,
        not args.anyskin_ascii,
        args.anyskin_stale_timeout_sec,
        args.anyskin_reconnect_delay_sec,
        args.anyskin_set_control_lines,
        args.anyskin_filter_alpha,
        args.anyskin_filter_release_alpha,
        args.anyskin_spike_step_limit,
    )
    print(f"Opening {args.port} @ {args.baudrate}. Servo IDs: {args.ids}")
    print(f"Range config: {args.config}")
    if tactile.enabled:
        print(f"AnySkin ports: {', '.join(tactile.ports)}")

    try:
        tactile.start()
    except Exception as exc:
        tactile.close()
        print(f"WARNING: AnySkin disabled: {exc}")
        tactile = AnySkinMonitor(
            [],
            args.anyskin_num_mags,
            args.anyskin_baudrate,
            args.contact_threshold,
            contact_thresholds,
            ignored_anyskin_indexes,
            args.contact_calibration_samples,
            args.anyskin_warmup_sec,
            args.anyskin_startup_timeout_sec,
            args.allow_partial_anyskin,
            False,
            args.anyskin_viz_rate_hz,
            not args.anyskin_ascii,
            args.anyskin_stale_timeout_sec,
            args.anyskin_reconnect_delay_sec,
            args.anyskin_set_control_lines,
            args.anyskin_filter_alpha,
            args.anyskin_filter_release_alpha,
            args.anyskin_spike_step_limit,
        )

    if args.sensor_only:
        print("Sensor-only mode. Commands:")
        print("  tactile | tactile_status | tactile_calibrate [samples] | tactile_rezero [samples]")
        print("  quit")
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                tactile.close()
                return 0
            if not line:
                continue
            parts = line.split()
            command = parts[0].lower()
            values = parts[1:]
            try:
                if command in ("quit", "exit", "q"):
                    tactile.close()
                    return 0
                if command in ("tactile", "touch"):
                    print(f"AnySkin strength: {format_strengths(tactile.strengths())}, threshold={tactile.threshold:.2f}")
                    continue
                if command in ("tactile_status", "touch_status"):
                    for status_line in tactile.status_lines():
                        print(status_line)
                    continue
                if command in ("tactile_stream", "touch_stream"):
                    seconds = float(values[0]) if len(values) >= 1 else 30.0
                    hz = float(values[1]) if len(values) >= 2 else 5.0
                    delay = 1.0 / max(hz, 0.1)
                    deadline = time.monotonic() + seconds
                    while time.monotonic() < deadline:
                        strengths = tactile.strengths()
                        contact = tactile.contact()
                        print(
                            f"AnySkin strength: {format_strengths(strengths)} "
                            f"contact={contact}"
                        )
                        time.sleep(delay)
                    continue
                if command in ("tactile_calibrate", "touch_calibrate", "tactile_rezero", "touch_rezero", "rezero"):
                    samples = int(values[0]) if values else args.contact_calibration_samples
                    tactile.calibrate(samples)
                    continue
                print("unknown command")
            except Exception as exc:
                print(f"ERROR: {exc}")

    try:
        with open_bus(args.port, args.baudrate, timeout=0.08) as ser:
            print("Connected. Commands:")
            print("  read")
            print("  scan [start] [end]")
            print("  move <id> <raw> [speed] [acc]")
            print("  pair <raw1> <raw2> [speed] [acc]")
            print("  save_open | save_close")
            print("  open [speed] [acc] | close [speed] [acc] | close_safe [speed] [acc]")
            print("  tactile | tactile_status | tactile_stream [seconds] [hz]")
            print("  tactile_calibrate [samples] | tactile_rezero [samples]")
            print("  empty_close_calibrate [speed] [acc]")
            print("  stream [seconds] [hz]")
            print("  quit")

            print_positions("current", read_positions(ser, args.ids))

            while True:
                try:
                    line = input("> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    return 0
                if not line:
                    continue

                parts = line.split()
                command = parts[0].lower()
                values = parts[1:]

                try:
                    if command in ("quit", "exit", "q"):
                        return 0

                    if command == "read":
                        print_positions("current", read_positions(ser, args.ids))
                        continue

                    if command == "scan":
                        start = int(values[0]) if len(values) >= 1 else 0
                        end = int(values[1]) if len(values) >= 2 else 20
                        found = []
                        for servo_id in range(start, end + 1):
                            if ping(ser, servo_id):
                                found.append(servo_id)
                        print("found:", found if found else "none")
                        continue

                    if command == "move":
                        if len(values) < 2:
                            print("use: move <id> <raw> [speed] [acc]")
                            continue
                        servo_id = int(values[0])
                        raw = int(values[1])
                        speed, acc = parse_speed_acc(values[2:], args.speed, args.acc)
                        ok = write_position(ser, servo_id, raw, speed, acc)
                        print(f"move id={servo_id} raw={raw} speed={speed} acc={acc} ok={ok}")
                        continue

                    if command == "pair":
                        if len(values) < len(args.ids):
                            print(f"use: pair {' '.join(f'<raw{i+1}>' for i in range(len(args.ids)))} [speed] [acc]")
                            continue
                        raws = [int(value) for value in values[: len(args.ids)]]
                        speed, acc = parse_speed_acc(values[len(args.ids) :], args.speed, args.acc)
                        send_pair(ser, args.ids, raws, speed, acc)
                        continue

                    if command in ("save_open", "save_close"):
                        key = "open" if command == "save_open" else "close"
                        positions = read_positions(ser, args.ids)
                        if any(position is None for position in positions.values()):
                            print_positions("cannot save, missing position", positions)
                            continue
                        config[key] = {str(servo_id): positions[servo_id] for servo_id in args.ids}
                        save_config(args.config, config)
                        print_positions(f"saved {key}", positions)
                        continue

                    if command in ("open", "close"):
                        raws = preset_raws(command, args.ids, config)
                        if raws is None:
                            print(f"No saved {command} pose. Move gripper there and run save_{command}.")
                            continue
                        if command == "open":
                            default_speed, default_acc = DEFAULT_OPEN_SPEED, DEFAULT_OPEN_ACC
                        else:
                            default_speed, default_acc = DEFAULT_CLOSE_SPEED, DEFAULT_CLOSE_ACC
                        speed, acc = parse_speed_acc(values, default_speed, default_acc)
                        send_pair(ser, args.ids, raws, speed, acc, label=command)
                        if command == "open" and tactile.enabled and args.rezero_after_open:
                            time.sleep(0.5)
                            try:
                                tactile.calibrate(args.open_rezero_samples)
                                print(f"AnySkin rezeroed after open with {args.open_rezero_samples} samples.")
                            except Exception as exc:
                                print(f"WARNING: AnySkin rezero after open failed: {exc}")
                        continue

                    if command in ("empty_close_calibrate", "tactile_empty_calibrate"):
                        speed, acc = parse_speed_acc(values, args.speed, args.acc)
                        calibrate_empty_close_baseline(
                            ser,
                            args.ids,
                            config,
                            args.config,
                            tactile,
                            speed,
                            acc,
                            args.safe_close_step_raw,
                            args.safe_close_interval_sec,
                        )
                        continue

                    if command == "close_safe":
                        close_raws = preset_raws("close", args.ids, config)
                        open_raws = preset_raws("open", args.ids, config)
                        if close_raws is None or open_raws is None:
                            print("Need saved open and close poses. Run save_open and save_close first.")
                            continue
                        config_for_safe_close = {
                            **config,
                            "close": {str(servo_id): raw for servo_id, raw in zip(args.ids, close_raws)},
                            "open": {str(servo_id): raw for servo_id, raw in zip(args.ids, open_raws)},
                        }
                        speed, acc = parse_speed_acc(values, args.speed, args.acc)
                        close_until_contact(
                            ser,
                            args.ids,
                            config_for_safe_close,
                            tactile,
                            speed,
                            acc,
                            args.safe_close_step_raw,
                            args.safe_close_interval_sec,
                            args.safe_stop_backoff_raw,
                            args.safe_hold_speed,
                            args.safe_hold_acc,
                            args.safe_hold_repeats,
                            args.safe_contact_confirm_sec,
                            args.safe_empty_baseline_margin,
                            args.safe_contact_confirm_samples,
                        )
                        continue

                    if command in ("tactile", "touch"):
                        print(f"AnySkin strength: {format_strengths(tactile.strengths())}, threshold={tactile.threshold:.2f}")
                        continue

                    if command in ("tactile_status", "touch_status"):
                        for status_line in tactile.status_lines():
                            print(status_line)
                        continue

                    if command in ("tactile_stream", "touch_stream"):
                        seconds = float(values[0]) if len(values) >= 1 else 30.0
                        hz = float(values[1]) if len(values) >= 2 else 5.0
                        delay = 1.0 / max(hz, 0.1)
                        deadline = time.monotonic() + seconds
                        while time.monotonic() < deadline:
                            strengths = tactile.strengths()
                            contact = tactile.contact()
                            print(
                                f"AnySkin strength: {format_strengths(strengths)} "
                                f"contact={contact}"
                            )
                            time.sleep(delay)
                        continue

                    if command in ("tactile_calibrate", "touch_calibrate", "tactile_rezero", "touch_rezero", "rezero"):
                        samples = int(values[0]) if values else args.contact_calibration_samples
                        tactile.calibrate(samples)
                        continue

                    if command == "stream":
                        seconds = float(values[0]) if len(values) >= 1 else 10.0
                        hz = float(values[1]) if len(values) >= 2 else 5.0
                        delay = 1.0 / max(hz, 0.1)
                        deadline = time.monotonic() + seconds
                        while time.monotonic() < deadline:
                            print_positions("current", read_positions(ser, args.ids))
                            if tactile.enabled:
                                print(f"AnySkin strength: {format_strengths(tactile.strengths())}")
                            time.sleep(delay)
                        continue

                    print("unknown command")
                except Exception as exc:
                    print(f"ERROR: {exc}")
    finally:
        tactile.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
