"""Source-clock measurements; never infer delay by shifting trajectories."""

import math
import statistics


def stats(values):
    values = sorted(values)
    if not values:
        return {"count": 0, "median_ms": None, "p95_ms": None, "max_ms": None}
    return {"count": len(values), "median_ms": statistics.median(values),
            "p95_ms": values[max(0, math.ceil(0.95 * len(values)) - 1)], "max_ms": values[-1]}


def summarize(frames, joint_names):
    durations = {key: [] for key in ("controller_read", "servoJ_call", "read_end_to_servoJ_return")}
    for frame in frames:
        for key, start, end in (
            ("controller_read", "read_start_ns", "read_end_ns"),
            ("servoJ_call", "send_start_ns", "send_end_ns"),
            ("read_end_to_servoJ_return", "read_end_ns", "send_end_ns"),
        ):
            if frame.get("send_ok") and frame.get(start) is not None and frame.get(end) is not None:
                duration = (frame[end] - frame[start]) / 1e6
                if duration >= 0:
                    durations[key].append(duration)
    # Detect isolated command departures after at least 300 ms of rest.
    # Values are feedback-observation times on the host, not robot actuation times.
    events = []
    threshold = math.radians(0.1)
    command_threshold = math.radians(0.25)
    for index, name in enumerate(joint_names):
        anchor = None
        rest_since = None
        pending = None
        previous = None
        for frame in frames:
            if not frame.get("send_ok") or not frame.get("robot_sample_consistent"):
                anchor = rest_since = pending = previous = None
                continue
            if previous and frame["robot_stamp"] <= previous["robot_stamp"]:
                continue
            if previous and frame["observed_ns"] - previous["observed_ns"] > 100_000_000:
                anchor = rest_since = pending = previous = None
            command = frame["command_q"][index]
            actual = frame["robot_q"][index]
            if pending:
                direction = pending["direction"]
                if direction * (command - pending["command_base"]) < command_threshold / 2:
                    pending = None
                    anchor = None
                elif frame["observed_ns"] - pending["start_ns"] > 2_000_000_000:
                    pending = None
                    anchor = None
                elif direction * (actual - pending["actual_base"]) >= threshold:
                    events.append({"joint": name, "command_seq": pending["seq"],
                                   "controller_read_start_ns": pending["read_start_ns"],
                                   "controller_read_end_ns": pending["read_end_ns"],
                                   "command_send_start_ns": pending["start_ns"],
                                   "input_read_end_to_send_ms": (
                                       (pending["start_ns"] - pending["read_end_ns"]) / 1e6
                                       if pending["read_end_ns"] is not None
                                       and pending["start_ns"] >= pending["read_end_ns"] else None),
                                   "robot_threshold_lower_s": previous["robot_stamp"],
                                   "robot_threshold_upper_s": frame["robot_stamp"],
                                   "pc_observed_lower_ns": previous["observed_ns"],
                                   "pc_observed_upper_ns": frame["observed_ns"],
                                   "observation_lower_ms": max(0, (previous["observed_ns"] - pending["start_ns"]) / 1e6),
                                   "observation_upper_ms": (frame["observed_ns"] - pending["start_ns"]) / 1e6,
                                   "threshold_deg": 0.1})
                    pending = None
                    anchor = None
            if pending is None:
                if anchor is None:
                    anchor = (command, actual)
                    rest_since = frame["send_start_ns"]
                elif abs(command - anchor[0]) >= command_threshold:
                    if (frame["send_start_ns"] - rest_since >= 300_000_000
                            and abs(actual - anchor[1]) < threshold):
                        pending = {"start_ns": frame["send_start_ns"], "seq": frame["seq"],
                                   "read_start_ns": frame.get("read_start_ns"),
                                   "read_end_ns": frame.get("read_end_ns"),
                                   "direction": 1 if command > anchor[0] else -1,
                                   "command_base": anchor[0], "actual_base": actual}
                    anchor = None
                elif abs(actual - anchor[1]) >= threshold:
                    anchor = (command, actual)
                    rest_since = frame["send_start_ns"]
            previous = frame
    return {"status": "measured" if frames else "not_recorded",
            "physical_one_way_latency_ms": None,
            "physical_latency_status": "not_measured_no_synchronized_actuation_timestamp",
            "clock": "source process time.monotonic_ns",
            "software_durations": {key: stats(value) for key, value in durations.items()},
            "response_events": events,
            "response_definition": "Host first-observation interval for 0.1 degree displacement after an isolated 0.25 degree command departure; includes feedback transport and polling. Not physical onset latency.",
            "software_definition": "servoJ call duration is SDK call wall time, not motion completion or one-way network latency."}


TIMESTAMP_FIELDS = [
    "command_seq", "joint", "controller_read_start_ns", "controller_read_end_ns",
    "command_send_start_ns", "command_send_end_ns", "command_send_ok",
    "robot_sample_time_s", "pc_observed_ns", "robot_sample_consistent",
    "controller_input_deg", "controller_aligned_target_deg", "command_deg", "robot_actual_deg",
]


def timestamp_rows(frames, joint_names):
    """Export unresampled samples. Host nanoseconds and robot seconds are separate clocks."""
    def angle(frame, field, index):
        values = frame.get(field, [])
        return math.degrees(values[index]) if index < len(values) else None

    for frame in frames:
        for index, name in enumerate(joint_names):
            yield dict(zip(TIMESTAMP_FIELDS, (
                frame.get("seq"), name, frame.get("read_start_ns"), frame.get("read_end_ns"),
                frame.get("send_start_ns"), frame.get("send_end_ns"), frame.get("send_ok"),
                frame.get("robot_stamp"), frame.get("observed_ns"), frame.get("robot_sample_consistent"),
                angle(frame, "controller_q", index), angle(frame, "controller_target_q", index),
                angle(frame, "command_q", index), angle(frame, "robot_q", index),
            )))
