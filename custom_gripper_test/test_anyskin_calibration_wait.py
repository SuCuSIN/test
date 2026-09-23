import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from manual_gripper_shell import AnySkinMonitor


class CalibrationWaitTests(unittest.TestCase):
    def run_calibration(self, stalled=False, invalid=False, cancel=False):
        clock = [0.0]
        streams = []
        for index in range(2):
            value = float('nan') if invalid and index == 1 else index + 1
            streams.append(SimpleNamespace(
                sample_cnt=0,
                get_data=lambda num_samples, v=value: [[0, v, v, v]] * num_samples))
        monitor = SimpleNamespace(enabled=True, calibration_samples=3,
                                  streams=streams, ports=['one', 'two'],
                                  _np=np, _sample_lock=threading.Lock(),
                                  baselines=['unchanged'])

        def sleep(seconds):
            clock[0] += seconds
            streams[0].sample_cnt += 1
            if not stalled:
                streams[1].sample_cnt += 1

        with patch('manual_gripper_shell.time.monotonic', side_effect=lambda: clock[0]), \
                patch('manual_gripper_shell.time.sleep', side_effect=sleep):
            if cancel:
                result = AnySkinMonitor.calibrate(
                    monitor, 3, cancel_requested=lambda: clock[0] >= 0.01)
                self.assertFalse(result)
                self.assertEqual(monitor.baselines, ['unchanged'])
                self.assertEqual(streams[0].sample_cnt, 1)
            elif stalled or invalid:
                with self.assertRaises(RuntimeError):
                    AnySkinMonitor.calibrate(monitor, 3)
                self.assertEqual(monitor.baselines, ['unchanged'])
            else:
                AnySkinMonitor.calibrate(monitor, 3)
                self.assertEqual([s.sample_cnt for s in streams], [3, 3])
                np.testing.assert_equal(monitor.baselines, [[1]*3, [2]*3])

    def test_waits_once_for_both_sensors(self):
        self.run_calibration()

    def test_stalled_sensor_does_not_replace_baseline(self):
        self.run_calibration(stalled=True)

    def test_nan_does_not_replace_baseline(self):
        self.run_calibration(invalid=True)

    def test_cancel_keeps_previous_baseline(self):
        self.run_calibration(cancel=True)

    def test_automatic_zero_preserves_contact_reference_and_filter(self):
        monitor = SimpleNamespace(
            enabled=True, calibration_samples=3, ports=['one'], _np=np,
            _sample_lock=threading.Lock(), baselines=[[10, 20, 30]],
            _filtered_magnet_strengths=[[9]],
            streams=[SimpleNamespace(sample_cnt=0,
                get_data=lambda num_samples: [[0, 100, 200, 300]] * num_samples,
                model_data=lambda samples: [[0, 40, 50, 60]] * samples)])
        def sleep(_):
            monitor.streams[0].sample_cnt += 1
        with patch('manual_gripper_shell.time.sleep', side_effect=sleep):
            self.assertTrue(AnySkinMonitor.calibrate(
                monitor, 3, preserve_contact_baseline=True))
        self.assertEqual(monitor.baselines, [[10, 20, 30]])
        self.assertEqual(monitor._filtered_magnet_strengths, [[9]])
        self.assertEqual(monitor.model_baselines, [[40, 50, 60]])

    def test_bounded_zero_accepts_small_drift(self):
        values = [[0, 5, 0, 0]] * 3
        stream = SimpleNamespace(sample_cnt=0, get_data=lambda num_samples: values)
        monitor = SimpleNamespace(enabled=True, calibration_samples=3, ports=['one'],
            _np=np, _sample_lock=threading.Lock(), streams=[stream],
            baselines=[[0, 0, 0]], contact_zero_reference=[[0, 0, 0]])
        def sleep(_):
            stream.sample_cnt += 1
        with patch('manual_gripper_shell.time.sleep', side_effect=sleep):
            self.assertTrue(AnySkinMonitor.calibrate(monitor, 3, contact_drift_limit=20))
            np.testing.assert_equal(monitor.baselines, [[5, 0, 0]])

    def test_stable_open_tare_accepts_large_residual_but_rejects_motion(self):
        values = [[0, 72, 0, 0]] * 3
        stream = SimpleNamespace(sample_cnt=0, get_data=lambda num_samples: values)
        monitor = SimpleNamespace(enabled=True, calibration_samples=3, ports=['one'],
            _np=np, _sample_lock=threading.Lock(), streams=[stream],
            baselines=[[0, 0, 0]], contact_zero_reference=[[0, 0, 0]])
        def sleep(_):
            stream.sample_cnt += 1
        with patch('manual_gripper_shell.time.sleep', side_effect=sleep):
            self.assertTrue(AnySkinMonitor.calibrate(monitor, 3, contact_stability_limit=20))
            np.testing.assert_equal(monitor.baselines, [[72, 0, 0]])
            self.assertEqual(monitor.contact_zero_reference, [[0, 0, 0]])
            values[:] = [[0, 0, 0, 0], [0, 72, 0, 0], [0, 150, 0, 0]]
            self.assertFalse(AnySkinMonitor.calibrate(monitor, 3, contact_stability_limit=20))
            np.testing.assert_equal(monitor.baselines, [[72, 0, 0]])
            values[:] = [[0, 24, 0, 0]] * 3
            self.assertFalse(AnySkinMonitor.calibrate(monitor, 3, contact_drift_limit=20))
            np.testing.assert_equal(monitor.baselines, [[72, 0, 0]])
            self.assertEqual(monitor.contact_zero_reference, [[0, 0, 0]])
            values[:] = [[0, -15, 0, 0], [0, 0, 0, 0], [0, 15, 0, 0]]
            self.assertFalse(AnySkinMonitor.calibrate(monitor, 3, contact_drift_limit=20))
            np.testing.assert_equal(monitor.baselines, [[72, 0, 0]])


if __name__ == '__main__':
    unittest.main()
