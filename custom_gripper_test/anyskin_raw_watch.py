"""Print raw AnySkin sample changes for a single sensor.

Use this when the contact strength stays at zero to check whether the sensor
data itself changes when the pad is pressed.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from manual_gripper_shell import AnySkinSerialStream


def finite(values: list[float]) -> list[float]:
    return [value for value in values if not math.isnan(value)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--num-mags", type=int, default=5)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--baseline-samples", type=int, default=50)
    parser.add_argument("--ascii", action="store_true")
    parser.add_argument("--set-control-lines", action="store_true")
    args = parser.parse_args()

    stream = AnySkinSerialStream(
        args.port,
        args.baudrate,
        args.num_mags,
        not args.ascii,
        set_control_lines=args.set_control_lines,
    )
    stream.start()
    try:
        print(f"Reading {args.port}. Keep the sensor unloaded for baseline...")
        deadline = time.monotonic() + 8.0
        while stream.sample_cnt < args.baseline_samples and time.monotonic() < deadline:
            time.sleep(0.02)

        samples = stream.get_data(args.baseline_samples)
        if len(samples) < 3:
            print("ERROR: no AnySkin samples received.")
            return 1
        baseline = np.median(np.asarray(samples, dtype=float)[:, 1:], axis=0)
        print(f"Baseline ready from {len(samples)} samples. Press the sensor now.")

        delay = 1.0 / max(args.hz, 0.1)
        stop_time = time.monotonic() + args.seconds
        while time.monotonic() < stop_time:
            sample = stream.get_data(1)
            if not sample:
                print("no sample")
                time.sleep(delay)
                continue
            values = np.asarray(sample[-1][1:], dtype=float)
            delta = values - baseline
            if delta.size % 3 == 0:
                per_mag = np.linalg.norm(delta.reshape((-1, 3)), axis=1)
                strength = float(np.max(per_mag))
            else:
                strength = float(np.linalg.norm(delta))
            preview = " ".join(f"{value:.1f}" for value in values[: min(6, len(values))])
            print(
                f"samples={stream.sample_cnt} strength={strength:.2f} "
                f"raw[0:6]=[{preview}] error={stream.last_error or 'none'}"
            )
            time.sleep(delay)
    finally:
        stream.terminate()
        stream.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
