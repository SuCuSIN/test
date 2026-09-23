import ast
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from gripper_position_tracking import PositionTracker
from gripper_regrasp import RegraspGuard, bounded_target_check, continuation_error


# Run the actual callback and target methods without importing ROS/hardware.
tree = ast.parse(Path(__file__).with_name('bluetooth_anyskin_gripper_shell.py').read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RosPositionGripperController')
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
           and n.name in ('_submit_probe', '_regrasp_target')]
scope = {'time': time, 'math': math, 'bounded_target_check': bounded_target_check,
         'continuation_error': continuation_error,
         'empty_baseline_magnet_strengths': lambda *args: [[0]*3]*2,
         'grouped_contact_deltas': lambda values, expected: values}
exec(compile(ast.Module(body=methods, type_ignores=[]), '<controller probe>', 'exec'), scope)


class ManualProbeTest(unittest.TestCase):
    def owner(self):
        obj = SimpleNamespace(condition=threading.Condition(), startup_calibration_ready=True,
                              await_open_after_calibration=False, manual_command=None,
                              args=SimpleNamespace(slip_regrasp='true', speed=2919, acc=22), latest_ratio=0,
                              probe_armed=False, probe_used=False, probe_pending=False,
                              regrasp=Mock(inhibited=False), regrasp_origin=None)
        obj.tracker = PositionTracker((2000, 2000), (1520, 2480), (2000, 2000), 2919)
        obj.tracker.command(0, time.monotonic())
        return obj

    def submit(self, obj, command):
        return scope['_submit_probe'](obj, command)

    def contact(self, obj):
        obj.latest_ratio = .6
        obj.tracker.command(.6, time.monotonic())
        obj.tracker.target = (1750, 2250)
        obj.tracker.contact = True
        obj.tracker.hold_input = .6

    def test_arm_only_open_and_one_request(self):
        obj = self.owner()
        self.assertTrue(self.submit(obj, 'probe_arm')['ok'])
        self.assertFalse(self.submit(obj, 'probe_once')['ok'])
        self.contact(obj)
        self.assertTrue(self.submit(obj, 'probe_once')['ok'])
        self.assertTrue(obj.probe_pending)
        self.assertFalse(self.submit(obj, 'probe_once')['ok'])
        self.assertFalse(self.submit(obj, 'probe_arm')['ok'])
        self.assertFalse(self.submit(obj, 'probe_off')['ok'])

    def test_opening_and_faulted_guard_block_probe(self):
        for opening in (True, False):
            obj = self.owner()
            self.submit(obj, 'probe_arm')
            self.contact(obj)
            if opening:
                obj.latest_ratio = 0
            else:
                obj.regrasp.inhibited = True
            self.assertFalse(self.submit(obj, 'probe_once')['ok'])
            self.assertFalse(obj.probe_pending)

    def test_armed_mode_suppresses_automatic_observe(self):
        obj = self.owner()
        self.submit(obj, 'probe_arm')
        self.contact(obj)
        obj._regrasp_motion_ready = Mock(return_value=True)
        self.assertIsNone(scope['_regrasp_target'](obj, time.monotonic(), [], [], True))
        obj.regrasp.observe.assert_not_called()

    def test_release_discards_pending_request(self):
        obj = self.owner()
        obj.probe_pending = obj.probe_used = True
        self.assertIsNone(scope['_regrasp_target'](obj, time.monotonic(), [], [], True))
        self.assertFalse(obj.probe_pending)
        self.assertTrue(obj.probe_used)

    def test_manual_target_rechecks_sensor_and_opening_after_read(self):
        for cancel, sensor_valid in ((False, True), (True, True), (False, False)):
            with self.subTest(cancel=cancel, sensor_valid=sensor_valid):
                obj = self.owner()
                obj.regrasp = RegraspGuard()
                obj.regrasp.observe = Mock(side_effect=AssertionError('automatic trigger used'))
                self.submit(obj, 'probe_arm')
                self.contact(obj)
                self.submit(obj, 'probe_once')
                obj.args.config_data = {}
                obj.args.safe_empty_baseline_margin = 20
                obj.web_target_active = False
                obj.latest_received = time.monotonic()
                obj.monitor = SimpleNamespace(ports=[1, 2], num_mags=3,
                    streams=[SimpleNamespace(last_sample_time=time.monotonic()) for _ in range(2)],
                    magnet_strengths=lambda: [[30 if sensor_valid else float('nan')]*3]*2)
                obj.stop_event = threading.Event()
                obj.node = Mock()
                obj._regrasp_motion_ready = Mock(return_value=True)
                def read(servo, phase, timeout):
                    if cancel:
                        obj.latest_ratio = 0
                    return (1750, 2250)[servo-1]
                obj._read_regrasp_position = read
                target = scope['_regrasp_target'](obj, time.monotonic(), [], [], True)
                self.assertEqual(target, (1744, 2256) if not cancel and sensor_valid else None)
                obj.regrasp.observe.assert_not_called()
                self.assertFalse(obj.probe_pending)

    def test_relaxed_contact_exception_is_manual_only(self):
        cases = ('manual', 'automatic', 'stale_sensor', 'invalid_sensor', 'opening',
                 'rise_cap', 'outside_limit', 'stale_command', 'shutdown')
        for case in cases:
            with self.subTest(case=case):
                obj = self.owner()
                obj.regrasp = RegraspGuard()
                self.submit(obj, 'probe_arm')
                self.contact(obj)
                self.submit(obj, 'probe_once')
                if case == 'automatic':
                    obj.probe_armed = False
                    obj.regrasp.observe = Mock(return_value=True)
                else:
                    obj.regrasp.observe = Mock(side_effect=AssertionError('automatic trigger used'))
                obj.args.config_data = {}
                obj.args.safe_empty_baseline_margin = 20
                obj.web_target_active = False
                obj.latest_received = time.monotonic() - (1 if case == 'stale_command' else 0)
                signal = float('nan') if case == 'invalid_sensor' else 0
                if case == 'rise_cap':
                    obj.regrasp.response_reference = [-100] * 6
                obj.monitor = SimpleNamespace(ports=[1, 2], num_mags=3,
                    streams=[SimpleNamespace(last_sample_time=time.monotonic() -
                                            (1 if case == 'stale_sensor' else 0)) for _ in range(2)],
                    magnet_strengths=lambda: [[signal]*3]*2)
                obj.stop_event = threading.Event()
                if case == 'shutdown':
                    obj.stop_event.set()
                obj.node = Mock()
                obj._regrasp_motion_ready = Mock(return_value=True)
                def read(servo, phase, timeout):
                    if case == 'opening':
                        obj.latest_ratio = 0
                    return (1500, 2500)[servo-1] if case == 'outside_limit' else (1750, 2250)[servo-1]
                obj._read_regrasp_position = read
                target = scope['_regrasp_target'](obj, time.monotonic(), [], [], True)
                self.assertEqual(target, (1744, 2256) if case == 'manual' else None)
                self.assertFalse(obj.probe_pending if case != 'automatic' else False)


if __name__ == '__main__':
    unittest.main()
