"""Independent buffered XYZ recorder; never sends serial or motor commands."""

import csv
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path


def xyz_fields(num_mags):
    return [f'm{i + 1}_{axis}' for i in range(num_mags) for axis in ('bx', 'by', 'bz')]


class AnySkinRawRecorder:
    def __init__(self, monitor, root):
        self.monitor = monitor
        self.path = Path(root) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.stop = threading.Event()
        self.thread = None
        self.error = ''
        self.stats = [{'port': stream.port, 'written': 0, 'dropped': 0, 'invalid': 0,
                       'start_sequence': stream.sample_cnt, 'last_sequence': stream.sample_cnt}
                      for stream in monitor.streams]

    def start(self):
        self.path.mkdir(parents=True, exist_ok=False)
        metadata = {
            'schema': 'anyskin_xyz_v1',
            'ports': list(self.monitor.ports),
            'num_mags': self.monitor.num_mags,
            'wall_time_s': time.time(),
            'pc_monotonic_s': time.monotonic(),
            'units': 'Decoded firmware magnetic units; NOT force or motion direction',
            'timestamps': 'Host receive timestamps, not sensor hardware timestamps',
            'processing': 'Unfiltered XYZ; no norm, channel grouping, or baseline subtraction',
            'calibration': 'Separate observed baseline snapshots; raw values never rezeroed',
            'scope': 'Samples after recorder start; earlier startup calibration not recorded',
        }
        (self.path / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        self.thread = threading.Thread(target=self._run, daemon=True, name='anyskin-raw-recorder')
        self.thread.start()
        print(f'AnySkin raw XYZ recording: {self.path}', flush=True)

    def drain(self, writer):
        for index, (stream, stats) in enumerate(zip(self.monitor.streams, self.stats)):
            rows, missing = stream.raw_samples_after(stats['last_sequence'])
            stats['dropped'] += missing
            for sequence, row in rows:
                stats['last_sequence'] = sequence
                if len(row) != 1 + 3 * self.monitor.num_mags or not all(math.isfinite(v) for v in row):
                    stats['invalid'] += 1
                    continue
                writer.writerow([index + 1, sequence, *row])
                stats['written'] += 1

    def _run(self):
        try:
            with (self.path / 'raw_xyz.csv').open('w', newline='', encoding='utf-8') as file, \
                    (self.path / 'calibration.jsonl').open('w', encoding='utf-8') as calibration:
                writer = csv.writer(file)
                writer.writerow(['sensor_index', 'sample_sequence', 'received_pc_monotonic_s',
                                 *xyz_fields(self.monitor.num_mags)])
                last_epoch = None
                last_flush = time.monotonic()
                while True:
                    snapshot = self.monitor.raw_calibration_snapshot()
                    epoch = snapshot['effective_pc_monotonic_s']
                    if epoch != last_epoch:
                        calibration.write(json.dumps(snapshot, allow_nan=False) + '\n')
                        calibration.flush()
                        last_epoch = epoch
                    self.drain(writer)
                    if time.monotonic() - last_flush >= 1:
                        file.flush()
                        last_flush = time.monotonic()
                    if self.stop.is_set():
                        break
                    self.stop.wait(0.02)
                self.drain(writer)
        except Exception as exc:
            self.error = str(exc)
            print(f'AnySkin raw recorder stopped (control unchanged): {exc}', flush=True)
        finally:
            try:
                (self.path / 'summary.json').write_text(json.dumps({
                    'sensors': self.stats, 'error': self.error,
                    'stopped_pc_monotonic_s': time.monotonic(),
                }, indent=2), encoding='utf-8')
            except Exception as exc:
                print(f'AnySkin raw summary could not be saved: {exc}', flush=True)

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                print('AnySkin raw writer still finishing; final flush not confirmed.', flush=True)
