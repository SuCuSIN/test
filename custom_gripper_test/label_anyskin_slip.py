"""Label live raw XYZ intervals without touching sensors or robot control."""

import argparse
import json
import math
import time
from pathlib import Path


LABELS = ('no_slip', 'down', 'up', 'left', 'right', 'press')


def validate_recording(folder):
    metadata_path = folder / 'metadata.json'
    if not metadata_path.is_file():
        raise ValueError('No metadata.json: start raw recording first')
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    if metadata.get('schema') != 'anyskin_xyz_v1':
        raise ValueError('Unsupported raw recording schema')
    if (folder / 'summary.json').exists():
        raise ValueError('This recording has ended; select a live recording')
    csv = folder / 'raw_xyz.csv'
    if not csv.is_file() or time.time() - csv.stat().st_mtime > 5:
        raise ValueError('Raw CSV is not updating; verify the recorder is running')
    return metadata


def make_event(label, start, end, pose, trial, completed):
    if label not in LABELS:
        raise ValueError('Unknown label')
    if not all(math.isfinite(v) for v in (start, end)) or end < start:
        raise ValueError('Invalid interval')
    return {
        'schema': 'anyskin_slip_label_v1', 'label': label,
        'start_pc_monotonic_s': start, 'end_pc_monotonic_s': end,
        'pose_id': pose, 'trial': trial, 'completed': completed,
        'direction_frame': 'operator view of sensor face at the named fixed pose',
        'source': 'human supplied interval; not automatically verified slip',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path,
                        help='Live raw-recording folder; defaults to latest folder')
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).parent / 'anyskin_raw_records')
    parser.add_argument('--seconds', type=float, default=3)
    parser.add_argument('--pose-id', default='pose_1')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 30:
        parser.error('--seconds must be between 1 and 30')
    folder = args.recording
    if folder is None:
        folders = sorted(args.root.glob('*/metadata.json'), key=lambda path: path.stat().st_mtime)
        if not folders:
            parser.error('No raw recordings found. Start the AnySkin control with raw recording enabled.')
        folder = folders[-1].parent
    try:
        validate_recording(folder)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f'Labels will be saved in: {folder / "slip_labels.jsonl"}')
    print('No motor commands. Keep a supported nonfragile object and fixed arm/camera pose.')
    print('n=contact without slip; d=down; u=up; l=left; r=right; p=press without sliding; q=quit')
    print('Directions are from your fixed view of the sensor face, NOT magnetic XYZ axes.')
    print('A label does not trigger regrasp. Disable automatic regrasp during data collection.')
    keys = dict(n='no_slip', d='down', u='up', l='left', r='right', p='press')
    trial = 0
    try:
        while True:
            key = input('Choose interval label, then Enter: ').strip().lower()
            if key == 'q':
                break
            if key not in keys:
                continue
            validate_recording(folder)
            trial += 1
            print(f'{keys[key]} starts after 2 seconds...', flush=True)
            time.sleep(2)
            start = time.monotonic()
            print('START', flush=True)
            complete = False
            try:
                time.sleep(args.seconds)
                complete = True
            finally:
                end = time.monotonic()
                try:
                    validate_recording(folder)
                except (ValueError, OSError):
                    complete = False
                event = make_event(keys[key], start, end, args.pose_id, trial, complete)
                with (folder / 'slip_labels.jsonl').open('a', encoding='utf-8') as file:
                    file.write(json.dumps(event) + '\n')
                print('END: ' + ('saved' if complete else 'incomplete; do not train on this interval'), flush=True)
    except KeyboardInterrupt:
        print('\nLabel collection stopped.')
    except (ValueError, OSError) as exc:
        print(f'Label collection stopped: {exc}')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
