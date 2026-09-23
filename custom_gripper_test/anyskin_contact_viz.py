"""Standalone AnySkin contact visualizer.

This keeps pygame on the main thread, which is more reliable under WSLg than
starting the visualizer from the interactive gripper shell input thread.
"""

from __future__ import annotations

import argparse

from manual_gripper_shell import (
    AnySkinMonitor,
    parse_float_list,
    parse_one_based_index_set,
    parse_ports,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ports", required=True, help="Comma-separated AnySkin ports")
    parser.add_argument("--num-mags", type=int, default=5)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--warmup-sec", type=float, default=1.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=8.0)
    parser.add_argument("--allow-partial", action="store_true")
    parser.add_argument("--viz-rate-hz", type=float, default=30.0)
    parser.add_argument("--ascii", action="store_true")
    parser.add_argument("--stale-timeout-sec", type=float, default=1.0)
    parser.add_argument("--reconnect-delay-sec", type=float, default=0.5)
    parser.add_argument("--set-control-lines", action="store_true")
    parser.add_argument("--filter-alpha", type=float, default=0.35)
    parser.add_argument("--filter-release-alpha", type=float, default=0.75)
    parser.add_argument("--spike-step-limit", type=float, default=80.0)
    parser.add_argument("--contact-threshold", type=float, default=110.0)
    parser.add_argument("--contact-thresholds", default="")
    parser.add_argument("--ignore-anyskin-indexes", default="")
    parser.add_argument("--calibration-samples", type=int, default=20)
    args = parser.parse_args()

    monitor = AnySkinMonitor(
        parse_ports(args.ports),
        args.num_mags,
        args.baudrate,
        args.contact_threshold,
        parse_float_list(args.contact_thresholds),
        parse_one_based_index_set(args.ignore_anyskin_indexes),
        args.calibration_samples,
        args.warmup_sec,
        args.startup_timeout_sec,
        args.allow_partial,
        False,
        args.viz_rate_hz,
        not args.ascii,
        args.stale_timeout_sec,
        args.reconnect_delay_sec,
        args.set_control_lines,
        args.filter_alpha,
        args.filter_release_alpha,
        args.spike_step_limit,
    )

    try:
        monitor.start()
        print("AnySkin visualizer running. Close the window or press Ctrl+C to exit.")
        monitor._visualizer_loop()
    except KeyboardInterrupt:
        print()
    finally:
        monitor.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
