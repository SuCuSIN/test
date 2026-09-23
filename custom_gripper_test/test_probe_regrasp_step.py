import unittest

from probe_regrasp_step import checked_target, midpoint_target, prepare_midpoint, read_diagnostic_pair
from unittest.mock import Mock


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.rows = [dict(actual_raw=1752, goal_raw=1753, torque_enable_raw=1, speed_raw=0),
                     dict(actual_raw=2244, goal_raw=2243, torque_enable_raw=1, speed_raw=0)]

    def target(self, rows):
        return checked_target(self.rows, rows, (2000, 2000), (1520, 2480))

    def test_reproduce_recorded_step(self):
        self.assertEqual(self.target(self.rows), (1750, 2246))

    def test_measured_six_uses_actual_not_goal(self):
        self.assertEqual(checked_target(self.rows, self.rows, (2000, 2000), (1520, 2480),
                                        mode='measured-6'), (1746, 2250))

    def test_measured_six_does_not_hide_unchanged_goal(self):
        rows = [dict(self.rows[0], actual_raw=1763, goal_raw=1760),
                dict(self.rows[1], actual_raw=2234, goal_raw=2240)]
        self.assertEqual(checked_target(rows, rows, (2000, 2000), (1520, 2480),
                                        mode='measured-6'), (1757, 2240))

    def test_measured_six_rejects_unresolved_pose(self):
        rows = [dict(self.rows[0], actual_raw=1767, goal_raw=1760), self.rows[1]]
        with self.assertRaises(ValueError):
            checked_target(rows, rows, (2000, 2000), (1520, 2480), mode='measured-6')

    def test_missing_diagnostic(self):
        with self.assertRaises(ValueError):
            self.target([None, self.rows[1]])

    def test_transient_missing_read_reacquires_both_jaws(self):
        client = Mock()
        old = dict(self.rows[1], actual_raw=2240)
        client.read_diagnostics.side_effect = [None, old, *self.rows]
        self.assertEqual(read_diagnostic_pair(client, Mock(), 'before'), self.rows)
        self.assertEqual([call.args[0] for call in client.read_diagnostics.call_args_list], [1, 2, 1, 2])
        client.sync_move.assert_not_called()

    def test_persistent_missing_read_is_bounded(self):
        client = Mock()
        client.read_diagnostics.return_value = None
        with self.assertRaises(RuntimeError):
            read_diagnostic_pair(client, Mock(), 'before')
        self.assertEqual(client.read_diagnostics.call_count, 6)
        client.sync_move.assert_not_called()

    def test_disconnect_does_not_reconnect(self):
        client = Mock()
        client.read_diagnostics.side_effect = ConnectionError('disconnected')
        with self.assertRaises(ConnectionError):
            read_diagnostic_pair(client, Mock(), 'before')
        client.connect.assert_not_called()
        client.sync_move.assert_not_called()

    def test_prepare_open_offset(self):
        rows = [dict(self.rows[0], actual_raw=1996), dict(self.rows[1], actual_raw=2006)]
        self.assertEqual(midpoint_target(rows, (2000, 2000), (1520, 2480)), (1760, 2240))

    def test_prepare_missing_blocks_motion(self):
        client = Mock()
        with self.assertRaises(ValueError):
            prepare_midpoint(client, lambda phase: [None, None], Mock(), (2000, 2000), (1520, 2480))
        client.sync_move.assert_not_called()

    def test_prepare_once_and_verify(self):
        client = Mock()
        rows = [dict(self.rows[0], actual_raw=1760, goal_raw=1760),
                dict(self.rows[1], actual_raw=2240, goal_raw=2240)]
        prepare_midpoint(client, lambda phase: rows, Mock(), (2000, 2000), (1520, 2480))
        client.sync_move.assert_called_once_with(1760, 2240, 80, 22)

    def test_recorded_stationary_error_allows_probe(self):
        rows = [dict(self.rows[0], actual_raw=1763, goal_raw=1760),
                dict(self.rows[1], actual_raw=2234, goal_raw=2240)]
        client = Mock()
        prepare_midpoint(client, lambda phase: rows, Mock(), (2000, 2000), (1520, 2480))
        client.sync_move.assert_called_once_with(1760, 2240, 80, 22)
        self.assertEqual(checked_target(rows, rows, (2000, 2000), (1520, 2480)), (1757, 2243))

    def test_stationary_error_over_six_rejected(self):
        rows = [dict(self.rows[0], actual_raw=1767, goal_raw=1760), self.rows[1]]
        with self.assertRaises(ValueError):
            checked_target(rows, rows, (2000, 2000), (1520, 2480))

    def test_reject_unsafe_or_changing_state(self):
        for change in (dict(actual_raw=2000), dict(actual_raw=1740),
                       dict(goal_raw=1750), dict(torque_enable_raw=0), dict(speed_raw=1)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.target([dict(self.rows[0], **change), self.rows[1]])


if __name__ == '__main__':
    unittest.main()
