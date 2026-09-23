import unittest
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from gripper_position_tracking import PositionTracker
from gripper_motion_timing import GripperMotionTiming
from bluetooth_anyskin_gripper_shell import BluetoothGripperClient, RosPositionGripperController


class TrackingTest(unittest.TestCase):
    def test_measured_open_releases_latch_and_next_close_checks_sensor(self):
        p = self.make()
        p.contact = p.blocked = True
        p.hold_input = .8
        p.command(0, 0)
        self.assertFalse(p.release_at_measured_open((1800, 2200), 0, 8))
        self.assertTrue(p.contact)
        self.assertTrue(p.release_at_measured_open(p.open, .01, 8))
        self.assertFalse(p.contact)
        self.assertFalse(p.blocked)
        self.assertIsNone(p.hold_input)
        p.command(.5, .02)
        self.assertIsNone(self.tick(p, .02, [[80]*3]*2))
        self.assertTrue(p.stop_requested)

    def test_open_pose_does_not_release_with_closing_or_stale_input(self):
        p = self.make()
        p.contact = True
        p.command(.5, 0)
        self.assertFalse(p.release_at_measured_open(p.open, 0, 8))
        p.command(0, 0)
        self.assertFalse(p.release_at_measured_open(p.open, 1, 8))
        self.assertTrue(p.contact)
    def test_reclose_after_offset_open_does_not_require_absolute_zero(self):
        p = self.make()
        p.acknowledge((1980, 2020))
        p.contact = True
        p.hold_input = 0.85
        p.command(0.7, 0)
        target = self.tick(p, 0)
        self.assertIsNotNone(target)
        self.assertGreater(p.offset, 0.5)
        p.acknowledge(p.open)
        p.command(0.1, 0.02)
        self.assertIsNone(self.tick(p, 0.02))
        p.command(0.1, 0.04)
        self.assertIsNone(self.tick(p, 0.04))
        p.command(0.12, 0.06)
        target = self.tick(p, 0.06)
        self.assertIsNotNone(target)
        self.assertLess(target[0], p.open[0])
        self.assertEqual(p.offset, 0)

    def test_reclose_after_offset_open_still_checks_contact(self):
        p = self.make()
        p.offset = 0.7
        p.command(0.1, 0)
        self.tick(p, 0)
        p.command(0.2, 0.02)
        self.assertIsNone(self.tick(p, 0.02, [[80]*3]*2))
        self.assertTrue(p.stop_requested)

    def test_first_candidate_requests_hold_before_confirmation(self):
        p = PositionTracker((2000, 2000), (1520, 2480), (1900, 2100),
                            2919, margin=20, confirm_sec=0.04,
                            confirm_samples=3, required_sensors=1)
        for i in range(4):
            p.command(1, i * 0.025)
            self.assertIsNone(p.tick(i * 0.025, 0.025, [[30], [0]], [i, i], True))
            self.assertEqual(p.stop_requested, i == 0)
            self.assertEqual(p.contact, i >= 2)
        p.command(0, 0.1)
        self.assertIsNotNone(p.tick(0.1, 0.025, [[30], [0]], [4, 4], True))
        self.assertFalse(p.stop_requested)
        self.assertFalse(p.contact)

    def test_candidate_hold_rearms_after_spike_clears(self):
        p = self.make()
        p.command(1, 0)
        self.tick(p, 0, [[80], [0]])
        self.assertTrue(p.stop_requested)
        self.assertFalse(p.contact)
        self.assertIsNotNone(self.tick(p, 0.02))
        self.assertFalse(p.stop_requested)
        self.tick(p, 0.04, [[80], [0]])
        self.assertTrue(p.stop_requested)

    def test_faster_tracking_keeps_step_limit_and_contact_pause(self):
        p = PositionTracker((2000, 2000), (1520, 2480), (2000, 2000), 1536)
        for dt in (0.002, 0.008, 0.02, 0.5):
            p.command(1, 0)
            target = p.tick(0, dt, [[0] * 3] * 2, [1, 1], True)
            self.assertIsNotNone(target)
            self.assertLessEqual(max(abs(a-b) for a, b in zip(target, p.target)), 8)
        self.assertIsNone(p.tick(0.008, 0.008, [[80] * 3] * 2, [2, 2], True))

    def make(self):
        return PositionTracker((2050, 1950), (1520, 2480), (2050, 1950), 280)

    def tick(self, tracker, t=0, channels=None, tokens=None, baseline=True):
        return tracker.tick(t, 0.02, channels or [[0] * 3] * 2,
                            tokens or [int(t * 1000)] * 2, baseline)

    def test_intermediate_position_and_direction(self):
        p = self.make()
        for i in range(100):
            p.command(0.4, i * 0.02)
            target = self.tick(p, i * 0.02)
            if target:
                self.assertLessEqual(max(abs(a-b) for a,b in zip(target,p.target)), 8)
                p.acknowledge(target)
        self.assertEqual(p.target, (1838, 2162))
        self.assertIsNone(self.tick(p, 1.99))
        p.command(0.2, 2)
        target = self.tick(p, 2)
        self.assertGreater(target[0], p.target[0])
        self.assertLess(target[1], p.target[1])

    def test_contact_pauses_then_latches_and_reversal_releases(self):
        p = self.make()
        p.acknowledge((1838, 2162))
        for i in range(8):
            p.command(0.9, i * 0.02)
            self.assertIsNone(self.tick(p, i * 0.02, [[80] * 3] * 2))
        self.assertTrue(p.contact)
        p.command(1, 0.2)
        self.assertIsNone(self.tick(p, 0.2))
        p.command(0.85, 0.22)
        target = self.tick(p, 0.22)
        self.assertGreater(target[0], p.target[0])
        self.assertFalse(p.contact)

    def test_one_sided_candidate_and_spike(self):
        p = self.make()
        p.command(1, 0)
        self.assertIsNone(self.tick(p, 0, [[80] * 3, [0] * 3]))
        self.assertFalse(p.contact)
        self.assertIsNotNone(self.tick(p, 0.04))

    def test_repeated_sample_cannot_confirm(self):
        p = self.make()
        for i in range(15):
            p.command(1, i * 0.02)
            self.tick(p, i * 0.02, [[80] * 3] * 2, [1, 1])
        self.assertFalse(p.contact)

    def test_contact_near_open_releases_even_below_motion_deadband(self):
        p = self.make()
        p.acknowledge((2049, 1951))
        p.contact = True
        p.hold_input = 0.15
        p.command(0.12, 0)
        self.tick(p, 0, [[100] * 3] * 2)
        self.assertFalse(p.contact)
        p.command(0, 0.02)
        target = self.tick(p, 0.02, [[100] * 3] * 2)
        if target:
            p.acknowledge(target)
        self.assertEqual(p.target, p.open)
        for i in range(1, 10):
            p.command(0, i * 0.02)
            self.tick(p, i * 0.02, [[100] * 3] * 2)
        self.assertFalse(p.contact)

    def test_open_endpoint_clears_contact_without_needing_more_travel(self):
        p = self.make()
        p.contact = p.blocked = True
        p.hold_input = 0.15
        p.command(0, 0)
        self.assertIsNone(self.tick(p, 0, [[100] * 3] * 2))
        self.assertFalse(p.contact)
        self.assertFalse(p.blocked)
        p.command(0.3, 0.02)
        self.assertIsNone(self.tick(p, 0.02, [[100] * 3] * 2))
        self.assertEqual(p.state, 'confirming contact; target held')

    def test_timeout_and_sensor_loss_require_opening(self):
        for loss in (True, False):
            p = self.make()
            p.acknowledge((1838, 2162))
            p.command(1, 0)
            self.assertIsNone(self.tick(p, 0 if loss else 0.6,
                                       [[float('nan')]] * 2 if loss else None))
            p.command(1, 0.7)
            self.assertIsNone(self.tick(p, 0.7))
            p.command(0.8, 0.8)
            self.assertIsNotNone(self.tick(p, 0.8))

    def test_missing_baseline_blocks_close_but_allows_open(self):
        p = self.make()
        p.acknowledge((1838, 2162))
        p.command(1, 0)
        self.assertIsNone(self.tick(p, baseline=False))
        p.command(0, 0.1)
        self.assertIsNotNone(self.tick(p, 0.1, baseline=False))

    def test_ros_callback_keeps_intermediate_width_and_latest_only(self):
        node = object.__new__(RosPositionGripperController)
        node.failure = None
        node.input_min_width, node.input_max_width = 0.012, 0.13
        node.condition = threading.Condition()
        node.manual_anchor = None
        node.startup_calibration_ready = True
        node.await_open_after_calibration = False
        node._command_callback(SimpleNamespace(data=[0.071]))
        self.assertAlmostEqual(node.latest_ratio, 0.5)
        node._command_callback(SimpleNamespace(data=[0.13]))
        self.assertEqual(node.latest_ratio, 0)

    def test_startup_gate_discards_close_until_calibration_and_open(self):
        node = object.__new__(RosPositionGripperController)
        node.failure = None
        node.input_min_width, node.input_max_width = .012, .13
        node.condition = threading.Condition()
        node.startup_calibration_ready = False
        node.await_open_after_calibration = False
        node.latest_ratio = None
        node.manual_anchor = None
        node._command_callback(SimpleNamespace(data=[.012]))
        self.assertIsNone(node.latest_ratio)
        node.startup_calibration_ready = True
        node.await_open_after_calibration = True
        node._command_callback(SimpleNamespace(data=[.012]))
        self.assertIsNone(node.latest_ratio)
        node._command_callback(SimpleNamespace(data=[.13]))
        self.assertFalse(node.await_open_after_calibration)
        self.assertEqual(node.latest_ratio, 0)
        node._command_callback(SimpleNamespace(data=[float('nan')]))
        self.assertEqual(node.latest_ratio, 0)

    def test_missing_position_prevents_startup_motion(self):
        node = object.__new__(RosPositionGripperController)
        node.client = SimpleNamespace(read_position=Mock(return_value=None))
        with self.assertRaisesRegex(RuntimeError, "Cannot read both"):
            node._initial_tracker()
        self.assertEqual(node.client.read_position.call_count, 6)

    def test_delayed_startup_position_is_retried(self):
        node = object.__new__(RosPositionGripperController)
        node.client = SimpleNamespace(read_position=Mock(side_effect=[None, 1950, 2050]))
        node.args = SimpleNamespace(config_data={"open": [2050, 1950], "close": [1520, 2480]},
                                    speed=280, safe_empty_baseline_margin=45,
                                    safe_contact_confirm_sec=0.1, safe_contact_confirm_samples=4,
                                    safe_contact_min_sensors=2)
        node.monitor = SimpleNamespace(ports=['a', 'b'], ignored_indexes=set())
        tracker = node._initial_tracker()
        self.assertEqual(tracker.target, (2050, 1950))
        self.assertEqual(node.client.read_position.call_count, 3)

    def test_worker_failure_is_exposed_to_supervising_process(self):
        node = object.__new__(RosPositionGripperController)
        node.condition = threading.Condition()
        node._set_feedback = Mock()
        node.node = SimpleNamespace(get_logger=lambda: Mock())
        node._initial_tracker = Mock(side_effect=RuntimeError('missing servo'))
        node._worker_loop()
        self.assertIn('missing servo', node.failure)
        self.assertIn('remain active', node.message)
        self.assertIsNone(node.latest_ratio)
        node._set_feedback.assert_called_once_with(False)
        node._command_callback(SimpleNamespace(data=[.012]))
        self.assertIsNone(node.latest_ratio)
        for command in ('open', 'close_safe', 'empty_close_calibrate'):
            self.assertFalse(node.submit(command)['ok'])

    def test_contact_hold_uses_measured_pose_not_outstanding_target(self):
        node = object.__new__(RosPositionGripperController)
        node.motion_timing = GripperMotionTiming()
        node.tracker = self.make()
        node.tracker.acknowledge((1700, 2300))
        node.client = SimpleNamespace(read_contact_positions=Mock(return_value=(1800, 2200)),
                                      sync_move=Mock())
        node.args = SimpleNamespace(hold_speed=20, hold_acc=1, firmware_hold="false")
        node.node = SimpleNamespace(get_logger=lambda: Mock())
        node._hold_contact_pose()
        node.client.read_contact_positions.assert_called_once_with()
        node.client.sync_move.assert_called_once_with(1800, 2200, 20, 1)
        self.assertEqual(node.tracker.target, (1800, 2200))

    def test_firmware_hold_does_not_send_followup_move(self):
        node = object.__new__(RosPositionGripperController)
        node.motion_timing = GripperMotionTiming()
        node.tracker = self.make()
        node.client = SimpleNamespace(hold=Mock(return_value=(1800, 2200)),
                                      read_position=Mock(), sync_move=Mock())
        node.args = SimpleNamespace(firmware_hold="true")
        node.node = SimpleNamespace(get_logger=lambda: Mock())
        node._hold_contact_pose()
        node.client.hold.assert_called_once_with()
        node.client.read_position.assert_not_called()
        node.client.sync_move.assert_not_called()
        self.assertEqual(node.tracker.target, (1800, 2200))

    def test_contact_hold_missing_pose_is_not_reported_as_success(self):
        node = object.__new__(RosPositionGripperController)
        node.motion_timing = GripperMotionTiming()
        node.client = SimpleNamespace(read_contact_positions=Mock(return_value=(None, None)), sync_move=Mock())
        node.args = SimpleNamespace(firmware_hold="false")
        with self.assertRaisesRegex(RuntimeError, 'motion state unknown'):
            node._hold_contact_pose()
        node.client.sync_move.assert_not_called()

    def test_full_open_rezero_requires_measured_pose_and_runs_once(self):
        node = object.__new__(RosPositionGripperController)
        node.tracker = self.make()
        node.tracker.command(0, 0)
        node.open_rezero_done = False
        node.open_rezero_check_at = 0
        node.latest_ratio = 0
        node.latest_received = time.monotonic()
        node.manual_command = None
        node.web_target_active = False
        node.condition = threading.Condition()
        node.stop_event = threading.Event()
        node.args = SimpleNamespace(open_rezero_samples=300, open_position_tolerance_raw=8,
                                    open_rezero_delay_sec=0, safe_empty_baseline_margin=20)
        node.monitor = SimpleNamespace(enabled=True, calibrate=Mock())
        node.node = SimpleNamespace(get_logger=lambda: Mock())
        node._set_feedback = Mock()
        node.client = SimpleNamespace(read_position=Mock(side_effect=[1900, 2100]))
        self.assertFalse(node._rezero_at_full_open(0))
        node.monitor.calibrate.assert_not_called()
        node.tracker.command(0, 1)
        node.client.read_position = Mock(return_value=2050)
        node.client.read_position.side_effect = lambda i, timeout: 2050 if i == 1 else 1950
        self.assertTrue(node._rezero_at_full_open(1))
        self.assertEqual(node.monitor.calibrate.call_args.args, (300,))
        self.assertIn('cancel_requested', node.monitor.calibrate.call_args.kwargs)
        self.assertEqual(node.monitor.calibrate.call_args.kwargs['contact_stability_limit'], 20)
        self.assertFalse(node._rezero_at_full_open(1.1))
        node.tracker.command(.2, 1.2)
        self.assertFalse(node._rezero_at_full_open(1.2))
        self.assertFalse(node.open_rezero_done)

    def test_single_required_sensor_confirms_and_reopens_with_sensor_still_high(self):
        p = self.make()
        p.required = 1
        p.acknowledge((1800, 2200))
        for i in range(8):
            p.command(0.9, i * .02)
            self.tick(p, i * .02, [[80]*3, [0]*3])
        self.assertTrue(p.contact)
        p.command(0, .2)
        self.assertIsNotNone(self.tick(p, .2, [[80]*3, [0]*3]))
        self.assertFalse(p.contact)

    def test_missing_ack_is_not_success(self):
        client = BluetoothGripperClient('127.0.0.1', 15555)
        with self.assertRaises(TimeoutError):
            client.wait_for_command_ack('SYNC_MOVE sent', timeout=0)

    def test_delayed_bluetooth_ack_is_accepted(self):
        client = BluetoothGripperClient('127.0.0.1', 15555)
        def reply():
            time.sleep(0.45)
            client.lines.put('SYNC_MOVE sent: id=1 logical=-55, id=2 logical=-39')
        worker = threading.Thread(target=reply)
        worker.start()
        try:
            client.wait_for_command_ack('SYNC_MOVE sent')
        finally:
            worker.join()

    def test_firmware_error_is_not_accepted_as_ack(self):
        client = BluetoothGripperClient('127.0.0.1', 15555)
        client.lines.put('ERROR: target exceeds configured gripper limits')
        with self.assertRaisesRegex(RuntimeError, 'target exceeds'):
            client.wait_for_command_ack('SYNC_MOVE sent')


if __name__ == '__main__':
    unittest.main()
