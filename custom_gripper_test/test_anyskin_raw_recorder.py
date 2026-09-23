import csv
import io
import json
import tempfile
import threading
import unittest
from types import SimpleNamespace

from anyskin_raw_recorder import AnySkinRawRecorder, xyz_fields
from manual_gripper_shell import AnySkinSerialStream, AnySkinMonitor


class RawTests(unittest.TestCase):
    def stream(self, capacity=1000):
        return AnySkinSerialStream('test-port', 115200, 2, True, max_samples=capacity)

    def monitor(self, stream):
        return SimpleNamespace(streams=[stream], ports=['test-port'], num_mags=2,
            raw_calibration_snapshot=lambda: {'effective_pc_monotonic_s': 1.0,
                                              'baseline_xyz_flat': [[0] * 6]})

    def test_snapshot_does_not_consume_or_mutate(self):
        stream = self.stream()
        stream._record([1, -2, 3, -4, 5, -6])
        before = stream.get_data(1)
        rows, missing = stream.raw_samples_after(0)
        rows[0][1][1] = 999
        self.assertEqual(stream.get_data(1), before)
        self.assertEqual(stream.sample_cnt, 1)
        self.assertEqual(missing, 0)

    def test_batch_gaps_counted(self):
        stream = self.stream(capacity=2)
        for i in range(5):
            stream._record([i] * 6)
        rows, missing = stream.raw_samples_after(0)
        self.assertEqual([item[0] for item in rows], [4, 5])
        self.assertEqual(missing, 3)

    def test_all_axes_signed_no_duplicate_records(self):
        stream = self.stream()
        recorder = AnySkinRawRecorder(self.monitor(stream), '.')
        stream._record([1, -2, 3, -4, 5, -6])
        output = io.StringIO()
        recorder.drain(csv.writer(output))
        recorder.drain(csv.writer(output))
        records = list(csv.reader(io.StringIO(output.getvalue())))
        self.assertEqual(len(records), 1)
        self.assertEqual([float(v) for v in records[0][3:]], [1, -2, 3, -4, 5, -6])

    def test_invalid_frames_counted(self):
        stream = self.stream()
        recorder = AnySkinRawRecorder(self.monitor(stream), '.')
        stream._record([float('nan')] * 6)
        stream._record([1])
        recorder.drain(csv.writer(io.StringIO()))
        self.assertEqual(recorder.stats[0]['invalid'], 2)

    def test_final_flush_files_and_baseline(self):
        stream = self.stream()
        with tempfile.TemporaryDirectory() as folder:
            recorder = AnySkinRawRecorder(self.monitor(stream), folder)
            recorder.start()
            stream._record([-1, 2, 3, 4, 5, 6])
            recorder.close()
            self.assertFalse(recorder.thread.is_alive())
            summary = json.loads((recorder.path / 'summary.json').read_text())
            self.assertEqual(summary['sensors'][0]['written'], 1)
            self.assertEqual(summary['error'], '')
            with (recorder.path / 'raw_xyz.csv').open() as file:
                records = list(csv.reader(file))
            self.assertEqual(records[0][3:], xyz_fields(2))
            self.assertEqual(records[1][3], '-1')
            self.assertTrue((recorder.path / 'calibration.jsonl').read_text())

    def test_calibration_snapshot_is_independent(self):
        monitor = AnySkinMonitor.__new__(AnySkinMonitor)
        monitor._sample_lock = threading.Lock()
        monitor.raw_calibration_monotonic = 2.0
        monitor.last_calibration_time = 3.0
        monitor.last_calibration_samples = 300
        monitor.ports = ['test-port']
        monitor.baselines = [[1, -2, 3]]
        monitor._filtered_magnet_strengths = [[9.0]]
        snapshot = monitor.raw_calibration_snapshot()
        snapshot['baseline_xyz_flat'][0][0] = 999
        self.assertEqual(monitor.baselines, [[1, -2, 3]])
        self.assertEqual(monitor._filtered_magnet_strengths, [[9.0]])

    def test_recording_failure_does_not_stop_stream_buffer(self):
        stream = self.stream()
        monitor = self.monitor(stream)
        def fail():
            raise OSError('simulated recording metadata failure')
        monitor.raw_calibration_snapshot = fail
        with tempfile.TemporaryDirectory() as folder:
            recorder = AnySkinRawRecorder(monitor, folder)
            recorder.start()
            recorder.close()
            self.assertIn('simulated', recorder.error)
            stream._record([1] * 6)
            self.assertEqual(stream.sample_cnt, 1)
            self.assertEqual(stream.get_data(1)[0][1:], [1] * 6)


if __name__ == '__main__':
    unittest.main()
