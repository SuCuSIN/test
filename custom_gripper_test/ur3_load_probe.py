"""Read-only CB3 load experiment. Never sends motion or payload commands."""

import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path


def summarize(rows):
    # Moving samples are retained in CSV but excluded from static statistics.
    static = [row for row in rows if row['stationary']]
    result = {'samples': len(rows), 'stationary_samples': len(static)}
    for name in ('fx', 'fy', 'fz', 'tx', 'ty', 'tz'):
        values = [row[name] for row in static]
        result[name] = {
            'median': statistics.median(values) if values else None,
            'stddev': statistics.pstdev(values) if values else None,
        }
    for i in range(6):
        values = [row[f'q{i + 1}'] for row in static]
        result[f'q{i + 1}_span_rad'] = max(values) - min(values) if values else None
    return result


def sample(receiver):
    before = receiver.getTimestamp()
    force = list(receiver.getActualTCPForce())
    q = list(receiver.getActualQ())
    qd = list(receiver.getActualQd())
    after = receiver.getTimestamp()
    if before != after:
        return None
    if any(len(values) != 6 for values in (force, q, qd)):
        raise ValueError('Incomplete RTDE sample')
    if not all(math.isfinite(v) for v in [after, *force, *q, *qd]):
        raise ValueError('Non-finite RTDE sample')
    row = {'pc_monotonic_s': time.monotonic(), 'robot_s': after}
    row.update(zip(('fx', 'fy', 'fz', 'tx', 'ty', 'tz'), force))
    row.update({f'q{i + 1}': v for i, v in enumerate(q)})
    row.update({f'qd{i + 1}': v for i, v in enumerate(qd)})
    row['stationary'] = max(abs(v) for v in qd) <= 0.01
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.50.14')
    parser.add_argument('--tool-mass-kg', type=float, default=0.5)
    parser.add_argument('--object-mass-kg', type=float, required=True)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'load_reports')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.tool_mass_kg, args.object_mass_kg, args.seconds)):
        parser.error('Values must be finite')
    if args.tool_mass_kg < 0 or args.object_mass_kg < 0 or args.seconds <= 0:
        parser.error('Mass must be nonnegative and duration positive')
    if args.tool_mass_kg + args.object_mass_kg > 3:
        parser.error('Declared total exceeds UR3 3 kg rating')
    from rtde_receive import RTDEReceiveInterface

    folder = args.output / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    metadata = {
        'tool_mass_kg_declared': args.tool_mass_kg,
        'object_mass_kg_known': args.object_mass_kg,
        'robot_ip': args.robot_ip,
        'note': 'Declared masses are NOT measured or applied to robot. '
                'TCP force is payload compensated. No weight estimator or haptics enabled. '
                'Compare static trials at the same pose with documented active payload/CoG.',
    }
    (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print('Read-only: no motor, torque, or payload commands. Ctrl+C saves results.', flush=True)
    print(f'Output: {folder}', flush=True)
    receiver = None
    rows = []
    try:
        receiver = RTDEReceiveInterface(args.robot_ip, 50.0)
        start = last_fresh = time.monotonic()
        last_stamp = None
        with (folder / 'samples.csv').open('w', newline='', encoding='utf-8') as file:
            writer = None
            while time.monotonic() - start < args.seconds:
                if not receiver.isConnected():
                    raise ConnectionError('RTDE disconnected')
                row = sample(receiver)
                if row is not None and row['robot_s'] != last_stamp:
                    if last_stamp is not None and row['robot_s'] < last_stamp:
                        raise RuntimeError('Robot clock reset; start a new trial')
                    last_stamp = row['robot_s']
                    last_fresh = time.monotonic()
                    if writer is None:
                        writer = csv.DictWriter(file, fieldnames=list(row))
                        writer.writeheader()
                    writer.writerow(row)
                    file.flush()
                    rows.append(row)
                if time.monotonic() - last_fresh > 1:
                    raise TimeoutError('No fresh RTDE data for 1 second')
                time.sleep(0.02)
    except KeyboardInterrupt:
        print('Recording stopped.')
    finally:
        result = summarize(rows)
        (folder / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2))
        if receiver is not None:
            receiver.disconnect()


if __name__ == '__main__':
    main()
