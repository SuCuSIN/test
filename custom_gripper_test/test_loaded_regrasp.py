import ast
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

import loaded_regrasp as ramp
from gripper_regrasp import bounded_target_check, continuation_error
from gripper_position_tracking import PositionTracker
from learned_slip_regrasp import LearnedRegraspGuard
import math


def diagnostic(servo, goal, actual, stamp=10, load=32):
    return dict(id=servo, ok=True, status=0, servo_status_raw=0, observed_pc_ns=int(stamp*1e9),
                mode_raw=0, torque_enable_raw=1, torque_limit_raw=1000,
                goal_raw=goal, actual_raw=actual, load_raw=load, current_raw=1)


class PlannerTests(unittest.TestCase):
    def test_logged_goal_ramps_beyond_unresolved_small_error(self):
        measured = (1751, 2244)
        previous = origin = (1751, 2246)
        goals = []
        for _ in range(4):
            target, error = ramp.next_target(measured, previous, origin, measured,
                                             (2000, 2000), (1520, 2480))
            self.assertIsNone(error)
            goals.append(target)
            previous = target
        self.assertEqual(goals, [(1748, 2249), (1745, 2252), (1742, 2255), (1739, 2256)])
        self.assertIsNone(ramp.next_target(measured, previous, origin, measured,
                                           (2000, 2000), (1520, 2480))[0])

    def test_responding_jaw_is_not_tightened_again(self):
        target, error = ramp.next_target((1748, 2244), (1748, 2249), (1751, 2246),
            (1751, 2244), (2000, 2000), (1520, 2480), (True, False))
        self.assertIsNone(error)
        self.assertEqual(target, (1748, 2252))

    def test_endpoint_and_grasp_budget_never_exceeded(self):
        for origin in ((1530, 2470), (1750, 2250)):
            previous = origin
            for _ in range(20):
                target, _ = ramp.next_target(previous, previous, origin, previous,
                    (2000, 2000), (1520, 2480))
                if target is None:
                    break
                self.assertTrue(1520 <= target[0] <= origin[0])
                self.assertTrue(origin[1] <= target[1] <= 2480)
                self.assertTrue(all(abs(v-a) <= 24 for v, a in zip(target, origin)))
                previous = target
            else:
                self.fail('unbounded reinforcement')

    def test_feedback_checks_goal_not_only_ack(self):
        records = [diagnostic(1, 1748, 1751), diagnostic(2, 2249, 2244, load=1072)]
        self.assertIsNone(ramp.feedback_error(records, (1748, 2249), 10.1, 9_000_000_000))
        self.assertEqual(ramp.signed_magnitude(1072, 10), -48)
        for field, value in (('goal_raw', 1751), ('torque_enable_raw', 0), ('mode_raw', 2),
                             ('servo_status_raw', 32), ('load_raw', 150), ('load_raw', 1174),
                             ('current_raw', 100), ('current_raw', 32868), ('observed_pc_ns', 0)):
            with self.subTest(field=field, value=value):
                bad = [dict(records[0], **{field: value}), records[1]]
                self.assertIsNotNone(ramp.feedback_error(bad, (1748, 2249), 10.1))
        self.assertIsNotNone(ramp.feedback_error([None, records[1]], (1748, 2249), 10.1))

    def test_episode_uses_confirmed_event_not_continuously_high_score(self):
        episode = dict(started=10, epoch=1)
        self.assertIsNone(ramp.episode_error(episode, (1, (11, 11), (.1, .1)), 11, 1))
        for now, epoch, prediction in ((13, 1, (1, (13, 13), (.9, .9))),
            (11, 2, (2, (11, 11), (.9, .9))), (11, 1, (1, (10, 10), (.9, .9)))):
            self.assertIsNotNone(ramp.episode_error(episode, prediction, now, epoch))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        tree = ast.parse(Path(__file__).with_name('bluetooth_anyskin_gripper_shell.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RosPositionGripperController')
        names = ('_regrasp_target', '_regrasp_motion_ready', '_record_regrasp_position')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        self.scope = dict(time=SimpleNamespace(monotonic=lambda: self.now), math=math,
            loaded_regrasp=ramp, bounded_target_check=bounded_target_check,
            continuation_error=continuation_error,
            empty_baseline_magnet_strengths=lambda *args: [[0]*3]*2,
            grouped_contact_deltas=lambda values, expected: values, json=__import__('json'))
        exec(compile(ast.Module(body=methods, type_ignores=[]), '<loaded controller>', 'exec'), self.scope)
        p = PositionTracker((2000, 2000), (1520, 2480), (1751, 2246), 2919)
        p.contact, p.hold_input = True, .6
        p.command(.6, self.now)
        g = LearnedRegraspGuard()
        g.attempts = 1
        g.observe = Mock(return_value=True)
        g.event_preflight_error = Mock(return_value=None)
        self.o = SimpleNamespace(tracker=p, regrasp=g, regrasp_origin=p.target,
            condition=threading.Condition(), probe_armed=False, latest_ratio=.6, latest_received=self.now,
            web_target_active=False, manual_command=None, stop_event=threading.Event(), node=Mock(),
            args=SimpleNamespace(slip_regrasp='true', regrasp_diagnostics='true', config_data={},
                                 safe_empty_baseline_margin=20, speed=2919, acc=22),
            monitor=SimpleNamespace(ports=[1, 2], num_mags=3, raw_calibration_monotonic=1,
                streams=[SimpleNamespace(last_sample_time=self.now) for _ in range(2)],
                magnet_strengths=lambda: [[30]*3]*2),
            slip_scorer=SimpleNamespace(result=(1, (self.now, self.now), (.9, .1))),
            regrasp_motor_records={}, String=lambda: SimpleNamespace(), timing_publisher=Mock())
        self.actual = [1751, 2244]
        self.o._read_regrasp_position = self.read
        self.o._regrasp_motion_ready = lambda t: self.scope['_regrasp_motion_ready'](self.o, t)

    def read(self, servo, phase, timeout):
        self.o.regrasp_motor_records[servo] = diagnostic(servo, self.o.tracker.target[servo-1],
                                                        self.actual[servo-1], self.now)
        return self.actual[servo-1]

    def advance(self, seconds):
        self.now += seconds
        self.o.latest_received = self.now
        self.o.tracker.command(self.o.latest_ratio, self.now)
        for stream in self.o.monitor.streams:
            stream.last_sample_time = self.now
        self.o.slip_scorer.result = (1, (self.now, self.now), (.1, .1))

    def target(self):
        return self.scope['_regrasp_target'](self.o, self.now, [[30]*3]*2, [1, 1], True)

    def test_missing_input_has_distinct_reason(self):
        self.o.tracker.received = None
        self.assertIsNone(self.target())
        self.assertEqual(self.o.last_regrasp_block['reason'], 'controller input missing')
        self.assertFalse(self.o.regrasp.inhibited)
        self.assertIsNotNone(self.o.regrasp.input_recovery)

    def test_timeout_is_not_reported_as_baseline_error(self):
        self.o.tracker.received = self.now-1
        self.assertIsNone(self.target())
        self.assertEqual(self.o.last_regrasp_block['reason'], 'controller input timeout')
        self.assertEqual(self.o.last_regrasp_block['controller_age_sec'], 1)

    def test_invalid_baseline_has_distinct_reason(self):
        self.assertIsNone(self.scope['_regrasp_target'](
            self.o, self.now, [[30]*3]*2, [1, 1], False))
        self.assertEqual(self.o.last_regrasp_block['reason'], 'invalid empty-close baseline')
        self.assertTrue(self.o.regrasp.inhibited)

    def test_timeout_discards_pending_work_without_resetting_budgets(self):
        self.o.probe_pending = True
        self.o.regrasp_continuation = {'step': 2}
        self.o.regrasp.response_reference = [25]*6
        origin = self.o.regrasp_origin
        self.o.tracker.received = self.now - 1
        self.assertIsNone(self.target())
        self.assertFalse(self.o.probe_pending)
        self.assertIsNone(self.o.regrasp_continuation)
        self.assertEqual(self.o.regrasp.attempts, 1)
        self.assertEqual(self.o.regrasp.response_reference, [25]*6)
        self.assertEqual(self.o.regrasp_origin, origin)
        self.advance(.01)
        self.assertIsNone(self.target())
        self.o.regrasp.observe.assert_not_called()

    def test_timeout_with_unverified_motion_does_not_auto_recover(self):
        self.acknowledge(self.target())
        self.o.tracker.received = self.now - 1
        self.assertIsNone(self.target())
        self.assertTrue(self.o.regrasp.inhibited)

    def acknowledge(self, target):
        o = self.o
        o.tracker.acknowledge_reinforcement(target)
        o.regrasp_position_trace = dict(before=o.regrasp_before_positions, target=target,
            ack_ns=int(self.now*1e9), attempt=1, loaded=True, trigger='automatic_slip',
            step=o.regrasp_step, started=o.regrasp_episode_started, epoch=1,
            motion_counts=[2 if s else 0 for s in o.regrasp_settled])

    def feedback(self):
        for servo in (1, 2):
            raw = self.read(servo, 'after', .1)
            self.scope['_record_regrasp_position'](self.o, servo, raw, int(self.now*1e9))

    def test_real_target_path_ramps_then_stops_on_encoder_response(self):
        goals = []
        for _ in range(3):
            target = self.target()
            self.assertIsNotNone(target)
            goals.append(target)
            self.acknowledge(target)
            self.advance(.21)
            self.feedback()
            self.advance(.21)
            self.feedback()
        self.assertEqual(goals, [(1748, 2249), (1745, 2252), (1742, 2255)])
        self.actual = [1749, 2246]
        self.advance(.01)
        self.feedback()
        self.advance(.01)
        self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))
        self.assertIn('displacement observed', self.o.regrasp_motion_result)
        self.assertFalse(self.o.regrasp.inhibited)
        self.assertEqual(self.o.regrasp.observe.call_count, 1)
        self.assertTrue(self.o.tracker.contact)
        self.assertEqual(self.o.tracker.hold_input, .6)

    def test_ramp_continuation_rechecks_opening_sensor_and_output(self):
        for case in ('opening', 'sensor_lost', 'contact_lost', 'nan', 'rise', 'overwritten', 'pwm', 'stale', 'expired'):
            with self.subTest(case=case):
                self.setUp()
                self.acknowledge(self.target())
                self.advance(.21)
                self.feedback()
                self.advance(.21)
                self.feedback()
                self.assertTrue(self.o._regrasp_motion_ready(self.now))
                if case == 'opening':
                    self.o.latest_ratio = 0
                elif case == 'sensor_lost':
                    self.o.monitor.streams[0].last_sample_time = 0
                elif case in ('contact_lost', 'nan'):
                    value = 0 if case == 'contact_lost' else float('nan')
                    self.o.monitor.magnet_strengths = lambda: [[value]*3]*2
                elif case == 'rise':
                    self.o.regrasp.response_reference = [-100]*6
                elif case == 'expired':
                    self.advance(3)
                else:
                    read = self.read
                    def altered(servo, phase, timeout):
                        result = read(servo, phase, timeout)
                        key, value = {'overwritten': ('goal_raw', 2000), 'pwm': ('load_raw', 200),
                                      'stale': ('observed_pc_ns', 0)}[case]
                        self.o.regrasp_motor_records[servo][key] = value
                        return result
                    self.o._read_regrasp_position = altered
                self.assertIsNone(self.target())

    def test_unresponsive_motor_does_not_cause_unbounded_commands(self):
        for _ in range(4):
            self.acknowledge(self.target())
            self.advance(.21)
            self.feedback()
            self.advance(.21)
            self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertTrue(self.o.regrasp.inhibited)
        self.assertIn('limit reached', self.o.regrasp_motion_result)

    def test_no_advance_from_only_one_feedback_sample(self):
        self.acknowledge(self.target())
        self.advance(.42)
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(hasattr(self.o, 'regrasp_continuation') and self.o.regrasp_continuation)

    def test_encoder_movement_does_not_hide_goal_overwrite(self):
        self.acknowledge(self.target())
        self.actual = [1749, 2246]
        for _ in range(2):
            self.advance(.21)
            self.feedback()
        self.o.regrasp_motor_records[1]['goal_raw'] = 2000
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertTrue(self.o.regrasp.inhibited)
        self.assertIn('goal readback differs', self.o.regrasp_motion_result)

    def test_stale_feedback_waits_for_read_without_new_command(self):
        self.acknowledge(self.target())
        self.advance(.21)
        self.feedback()
        self.advance(.4)
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(self.o.regrasp.inhibited)
        self.assertIsNotNone(self.o.regrasp_position_trace)
        self.assertFalse(getattr(self.o, 'regrasp_continuation', None))
        self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))

    def test_grace_observes_late_motion_but_never_advances_goal(self):
        self.acknowledge(self.target())
        self.advance(2.6)
        self.feedback()
        self.feedback()
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertFalse(getattr(self.o, 'regrasp_continuation', None))
        self.actual = [1749, 2246]
        self.advance(.1)
        self.feedback()
        self.advance(.1)
        self.feedback()
        self.assertTrue(self.o._regrasp_motion_ready(self.now))
        self.assertIn('displacement observed', self.o.regrasp_motion_result)

    def test_missing_feedback_still_expires(self):
        self.acknowledge(self.target())
        self.advance(3.3)
        self.assertFalse(self.o._regrasp_motion_ready(self.now))
        self.assertTrue(self.o.regrasp.inhibited)


if __name__ == '__main__':
    unittest.main()
