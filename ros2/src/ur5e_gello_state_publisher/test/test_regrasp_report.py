import unittest
from ur5e_gello_state_publisher.regrasp_report import episodes, table, slip_intervals
from ur5e_gello_state_publisher.tracking_report_recorder import make_gripper_svg


class RegraspReportTest(unittest.TestCase):
    def frame(self, stamp, score, contact=True, epoch=1):
        return dict(observed_pc_ns=int(stamp*1e9), contact=contact,
                    slip_contact_mask=[True, True],
                    slip_model_prediction=[epoch, [stamp, stamp], [score, .01]])

    def test_interval_starts_at_model_high_not_grasp(self):
        frames = [self.frame(t, s) for t, s in
                  ((1, .01), (2, .01), (2.1, .9), (2.2, .95),
                   (2.3, .01), (2.4, .01), (2.5, .01), (2.6, .01))]
        rows = slip_intervals(frames, 0)
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]['start_sec'], 2.1)
        self.assertAlmostEqual(rows[0]['end_sec'], 2.3)
        self.assertEqual(rows[0]['end_reason'], 'model cleared')
        svg = make_gripper_svg(4, [], [], {}, [], sensor_events=frames)
        self.assertIn('class="slip-interval"', svg)

    def test_gap_release_and_epoch_never_join_intervals(self):
        frames = [self.frame(1, .9), self.frame(1.1, .9),
                  self.frame(2, .9), self.frame(2.1, .9, epoch=2),
                  self.frame(2.2, .9, contact=False, epoch=2)]
        rows = slip_intervals(frames, 0)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]['end_sec'], 1.1)
        self.assertEqual(rows[-1]['end_reason'], 'contact released')

    def test_duplicate_low_samples_do_not_end_episode(self):
        high, low = self.frame(1, .9), self.frame(1.1, .01)
        rows = slip_intervals([high, low, low, low], 0)
        self.assertEqual(rows[0]['end_reason'], 'end not observed')

    def test_stages_are_one_episode_and_actual_is_not_command(self):
        event = dict(component='gripper_regrasp_event', episode_id=1,
                     observed_pc_ns=1000000000, trigger='automatic_slip',
                     previous_target_raw=[1757, 2241], closing_directions=[-1, 1])
        commands = [dict(regrasp=True, episode_id=1, send_start_ns=1100000000+i,
                         target_raw=target) for i, target in enumerate(
                             ([1754, 2244], [1745, 2253]))]
        observations = [dict(component='gripper_regrasp_position', episode_id=1,
                             observed_pc_ns=2000000000+i, servo_id=i,
                             closing_delta_raw=delta) for i, delta in ((1, 6), (2, 5))]
        rows = episodes([event] + observations, commands, 0)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['command_stages'], 2)
        self.assertEqual(rows[0]['id1_commanded_closing_raw'], 12)
        self.assertEqual(rows[0]['id1_observed_closing_raw'], 6)
        self.assertEqual(rows[0]['id2_observed_closing_raw'], 5)
        svg = make_gripper_svg(4, [], [], {}, [], reinforcement_events=rows)
        self.assertIn('Slip 1 @ 1.000s', svg)
        self.assertIn('ID1: 6 raw', svg)
        self.assertIn('Reinforcement command', svg)

    def test_cancelled_missing_and_manual_are_distinct(self):
        events = [dict(component='gripper_regrasp_event', episode_id=i,
                       observed_pc_ns=i*1000000000, trigger=trigger,
                       previous_target_raw=[1700, 2300], closing_directions=[-1, 1])
                  for i, trigger in ((1, 'automatic_slip'), (2, 'manual_probe'))]
        rows = episodes(events, [], 0)
        self.assertEqual(len(rows), 2)
        self.assertIsNone(rows[0]['id1_observed_closing_raw'])
        self.assertEqual(rows[0]['observation'], 'not commanded')
        self.assertEqual(rows[1]['trigger'], 'manual_probe')
        self.assertIn('Unavailable', table(rows))


if __name__ == '__main__':
    unittest.main()
