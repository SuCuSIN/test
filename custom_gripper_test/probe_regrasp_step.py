"""One explicitly confirmed unloaded step; no sensor or ROS controller started."""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

# Unloaded probe only: measured stationary errors were +3 and -6 counts.
PROBE_SETTLE_ERROR = 6


def read_diagnostic_pair(client, record, phase):
    # Retry reads only. Reacquire BOTH jaws so a missing reply is never replaced
    # by a cached position from before the operator's confirmation.
    for attempt in range(1, 4):
        pair = [client.read_diagnostics(i, timeout=0.75) for i in (1, 2)]
        record(phase, read_attempt=attempt, motors=pair)
        if all(item is not None for item in pair):
            return pair
        if attempt < 3:
            record('diagnostic_read_retry', phase=phase,
                   missing_ids=[i for i, item in enumerate(pair, 1) if item is None])
            time.sleep(0.1)
    raise RuntimeError(f'{phase}: diagnostics incomplete after 3 read attempts; no motion retry')


def midpoint_target(rows, opened, closed):
    if len(rows) != 2 or len(opened) != 2 or len(closed) != 2:
        raise ValueError('Two configured jaws required')
    targets = []
    for row, a, b in zip(rows, opened, closed):
        if row is None or row['torque_enable_raw'] != 1 or row['speed_raw'] != 0:
            raise ValueError('Preparation requires valid stationary, enabled motors')
        # Allow the small open-end offset observed in the diagnostic log.
        if a == b or not min(a, b) - 10 <= row['actual_raw'] <= max(a, b) + 10:
            raise ValueError('Preparation position outside configured travel')
        targets.append(round((a + b) / 2))
    return tuple(targets)


def prepare_midpoint(client, read_pair, record, opened, closed):
    target = midpoint_target(read_pair('prepare_before'), opened, closed)
    record('prepare_sending_once', target=target, speed=80, acc=22)
    client.sync_move(*target, 80, 22)
    deadline = time.monotonic() + 10
    settled = 0
    previous = None
    while time.monotonic() < deadline:
        rows = read_pair('prepare_position')
        if any(row is None for row in rows):
            raise RuntimeError('Preparation diagnostic missing; no reinforcement sent')
        reached = all(row['goal_raw'] == goal and abs(row['actual_raw'] - goal) <= PROBE_SETTLE_ERROR
                      and row['speed_raw'] == 0 and row['torque_enable_raw'] == 1
                      for row, goal in zip(rows, target))
        stable = previous is not None and all(abs(row['actual_raw'] - old['actual_raw']) <= 1
                                               for row, old in zip(rows, previous))
        settled = settled + 1 if reached and stable else 0
        previous = rows
        if settled >= 2:
            record('prepare_complete', target=target)
            return
        time.sleep(0.1)
    raise RuntimeError('Midpoint not reached within 10s; no reinforcement sent')


def checked_target(first, second, opened, closed, mode='previous-3'):
    if mode not in ('previous-3', 'measured-6'):
        raise ValueError('Unsupported probe mode')
    targets = []
    for index, (a, b) in enumerate(zip(opened, closed)):
        before, current = first[index], second[index]
        if before is None or current is None:
            raise ValueError('Missing motor diagnostics; no motion sent')
        value, goal = current['actual_raw'], current['goal_raw']
        if a == b or not 0.2 <= (value - a) / (b - a) <= 0.8:
            raise ValueError('Place BOTH empty jaws near mid travel using normal control first')
        if current['torque_enable_raw'] != 1 or current['speed_raw'] != 0:
            raise ValueError('Motor not enabled and stationary; no motion sent')
        if abs(value - before['actual_raw']) > 1 or goal != before['goal_raw']:
            raise ValueError('Position or goal changed; stop other controllers first')
        if abs(value - goal) > PROBE_SETTLE_ERROR:
            raise ValueError('Previous goal unresolved; no motion sent')
        direction = 1 if b > a else -1
        target = value + direction * 6 if mode == 'measured-6' else goal + direction * 3
        if direction * (target - goal) < 0:
            raise ValueError('Probe must not move the commanded goal toward open')
        if not min(a, b) <= target <= max(a, b) or not 0 < direction * (target - value) <= PROBE_SETTLE_ERROR + 3:
            raise ValueError('Small closing step cannot be issued at this position')
        targets.append(target)
    return tuple(targets)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=15555)
    parser.add_argument('--step-speed', type=int, choices=(80, 2919), default=80,
                        help='One-step speed; midpoint preparation remains at 80')
    parser.add_argument('--step-mode', choices=('previous-3', 'measured-6'), default='previous-3',
                        help='Previous goal +3 or measured position +6 closing counts')
    parser.add_argument('--prepare-midpoint', action='store_true',
                        help='Offer separately confirmed empty-jaw midpoint positioning')
    args = parser.parse_args()
    from bluetooth_anyskin_gripper_shell import (
        BluetoothGripperClient, DEFAULT_CONFIG, load_config, open_raws, close_raws,
    )
    config = load_config(DEFAULT_CONFIG)
    opened, closed = open_raws(config), close_raws(config)
    print('Stop ROS gripper control. Remove objects and keep fingers clear.')
    print('Connection may trigger existing firmware startup behavior. No automatic positioning.')
    if input('Type CONNECT to read diagnostics: ').strip() != 'CONNECT':
        return
    path = Path(__file__).parent / 'regrasp_probe_reports' / datetime.now().strftime('%Y%m%d_%H%M%S_%f.jsonl')
    path.parent.mkdir(parents=True, exist_ok=True)
    client = BluetoothGripperClient(args.host, args.port)
    with path.open('x', encoding='utf-8') as report:
        def record(event, **data):
            row = dict(event=event, pc_monotonic_s=time.monotonic(), **data)
            line = json.dumps(row, allow_nan=False)
            print(line, flush=True)
            report.write(line + '\n')
            report.flush()

        def read_pair(phase):
            return read_diagnostic_pair(client, record, phase)

        try:
            client.connect()
            time.sleep(1)
            read_pair('preview')
            if args.prepare_midpoint:
                print('Preparation moves both jaws halfway closed at speed 80, acc 22.')
                print('Remove ALL objects and fingers: no tactile stop is active.')
                if input('Type EMPTY MIDPOINT to prepare: ').strip() != 'EMPTY MIDPOINT':
                    return
                prepare_midpoint(client, read_pair, record, opened, closed)
            if args.step_mode == 'measured-6':
                print(f'One step only: measured position + 6 raw closing, speed={args.step_speed}, acc=22.')
                print('If existing goal is already 6 counts ahead, that jaw goal stays unchanged.')
            else:
                print(f'One step only: previous goal + 3 raw closing, speed={args.step_speed}, acc=22.')
                print('Unloaded probe tolerance=6 raw; actual-to-new-goal distance may be up to 9 raw.')
            print('No tactile stop in this standalone test. Empty gripper only.')
            if input('Type EMPTY STEP to proceed: ').strip() != 'EMPTY STEP':
                return
            first = read_pair('before_1')
            time.sleep(0.15)
            second = read_pair('before_2')
            target = checked_target(first, second, opened, closed, mode=args.step_mode)
            record('sending_once', target=target, speed=args.step_speed, acc=22, step_mode=args.step_mode,
                   before_actual=[p['actual_raw'] for p in second],
                   previous_goal=[p['goal_raw'] for p in second],
                   goal_advance=[(t - p['goal_raw']) * (1 if b > a else -1)
                                 for t, p, a, b in zip(target, second, opened, closed)])
            client.sync_move(*target, args.step_speed, 22)
            record('esp32_ack', target=target)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                pair = read_pair('after')
                if any(item is None for item in pair):
                    raise RuntimeError('Diagnostic response missing; result unknown')
                record('position_comparison', target=target,
                       goal_matches=[p['goal_raw'] == t for p, t in zip(pair, target)],
                       closing_delta=[(p['actual_raw'] - old['actual_raw']) * (1 if b > a else -1)
                                      for p, old, a, b in zip(pair, second, opened, closed)])
                time.sleep(0.05)
            record('complete', note='Encoder observations only; grip force not measured')
        except (Exception, KeyboardInterrupt) as exc:
            record('aborted', reason=str(exc), note='No retry/replay. Existing motor goal may remain active.')
            return 1
        finally:
            client.close()
            print(f'Report: {path}')
            print('Connection closed; this does NOT disable motor torque or cancel its goal.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
