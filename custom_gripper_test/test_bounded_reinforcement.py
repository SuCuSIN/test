import ast
import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from gripper_regrasp import RegraspGuard, continuation_error, bounded_target_check
from learned_slip_regrasp import LearnedRegraspGuard
from test_manual_regrasp_probe import ManualProbeTest, scope as target_scope


tree = ast.parse(Path(__file__).with_name('bluetooth_anyskin_gripper_shell.py').read_text(encoding='utf-8'))
controller = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RosPositionGripperController')
method = next(n for n in controller.body if isinstance(n, ast.FunctionDef) and n.name == '_regrasp_motion_ready')
scope = {}
exec(compile(ast.Module(body=[method], type_ignores=[]), '<bounded reinforcement>', 'exec'), scope)


class BoundedTests(unittest.TestCase):
    def test_continuation_runs_actual_target_checks(self):
        for case in ('valid', 'opening', 'sensor_stale', 'rise', 'model_stale', 'cleared', 'shutdown'):
            with self.subTest(case=case):
                factory = ManualProbeTest()
                o = factory.owner()
                factory.contact(o)
                now = time.monotonic()
                o.regrasp = LearnedRegraspGuard()
                o.regrasp.attempts = 1
                o.regrasp.observe = Mock(side_effect=AssertionError('new slip event requested'))
                o.regrasp.response_reference = [0 if case == 'rise' else 30] * 6
                o.regrasp_origin = (1753, 2247)
                o.regrasp_continuation = dict(started=now - 2, epoch=1)
                o._regrasp_motion_ready = Mock(return_value=True)
                o.args.config_data = {}
                o.args.safe_empty_baseline_margin = 20
                o.web_target_active = False
                o.latest_received = now
                o.monitor = SimpleNamespace(ports=[1, 2], num_mags=3, raw_calibration_monotonic=1,
                    streams=[SimpleNamespace(last_sample_time=now - (1 if case == 'sensor_stale' else 0))]*2,
                    magnet_strengths=lambda: [[100 if case == 'rise' else 30]*3]*2)
                stamp = now - (1 if case == 'model_stale' else 0)
                o.slip_scorer = SimpleNamespace(result=(1, (stamp, stamp),
                    (.1, .1) if case == 'cleared' else (.9, .1)))
                o.stop_event = threading.Event()
                if case == 'shutdown':
                    o.stop_event.set()
                o.node = Mock()
                def read(servo, phase, timeout):
                    if case == 'opening':
                        o.latest_ratio = 0
                    return (1750, 2250)[servo - 1]
                o._read_regrasp_position = read
                result = target_scope['_regrasp_target'](o, now, [], [], True)
                self.assertEqual(result, (1747, 2253) if case == 'valid' else None)
                self.assertEqual(o.regrasp.attempts, 1)
                self.assertIsNone(o.regrasp_continuation)

    def test_fresh_continuation_only(self):
        episode = dict(started=10, epoch=1)
        self.assertIsNone(continuation_error(episode, (1, (12, 12), (.9, .1)), 12, 1, (True, False), .8))
        for prediction in (None, (1, (11, 11), (.9, .1)), (1, (12, 12), (.1, .1)),
                           (1, (12, 12), (math.nan, .1)), (2, (12, 12), (.9, .1))):
            self.assertIsNotNone(continuation_error(episode, prediction, 12, 1, (True, False), .8))
        self.assertIsNotNone(continuation_error(episode, (1, (14, 14), (.9, .1)), 14, 1, (True, False), .8))
        self.assertIsNotNone(continuation_error(episode, (1, (12, 12), (.9, .1)), 12, 1, (False, True), .8))

    def owner(self, step=1, trigger='automatic_slip'):
        return SimpleNamespace(regrasp_position_trace=dict(ack_ns=10_000_000_000,
            trigger=trigger, step=step, started=10, epoch=1, motion_counts=[0, 0]),
            tracker=Mock(), regrasp=RegraspGuard(), node=Mock())

    def test_no_second_step_before_observation_window(self):
        o = self.owner()
        self.assertFalse(scope['_regrasp_motion_ready'](o, 11.9))
        self.assertFalse(hasattr(o, 'regrasp_continuation'))

    def test_one_continuation_then_inhibit(self):
        o = self.owner()
        self.assertTrue(scope['_regrasp_motion_ready'](o, 12))
        self.assertEqual(o.regrasp_continuation, dict(started=10, epoch=1))
        self.assertFalse(o.regrasp.inhibited)
        for step, trigger in ((2, 'automatic_slip'), (1, 'manual_probe')):
            o = self.owner(step, trigger)
            self.assertFalse(scope['_regrasp_motion_ready'](o, 12))
            self.assertTrue(o.regrasp.inhibited)
            self.assertFalse(hasattr(o, 'regrasp_continuation'))

    def test_observed_motion_never_queues_more(self):
        o = self.owner()
        o.regrasp_position_trace['motion_counts'] = [2, 2]
        self.assertTrue(scope['_regrasp_motion_ready'](o, 11))
        self.assertFalse(hasattr(o, 'regrasp_continuation'))

    def test_logged_stationary_jaws_get_second_goal_with_same_origin(self):
        args = dict(opened=(2000, 2000), closed=(1520, 2480), step=3,
                    advance_previous=True, tracking_tolerance=6, clamp_to_budget=True)
        origin = (1751, 2246)
        first, _ = bounded_target_check((1751, 2244), origin, origin, **args)
        second, error = bounded_target_check((1751, 2244), first, origin, **args)
        self.assertEqual(first, (1748, 2249))
        self.assertEqual(second, (1745, 2252))
        self.assertIsNone(error)


if __name__ == '__main__':
    unittest.main()
