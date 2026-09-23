import copy
import unittest

from learned_slip_regrasp import LearnedRegraspGuard
from slip_monitor_status import slip_monitor_status


class SlipMonitorTests(unittest.TestCase):
    def setUp(self):
        self.guard = LearnedRegraspGuard()
        self.guard.expected_epoch = 1
        self.prediction = (1, (10, 10), (.95, .1))

    def status(self, **changes):
        args = dict(guard=self.guard, prediction=self.prediction, error=None, epoch=1,
                    now=10.1, contact=True, enabled=True, probe=False)
        args.update(changes)
        return slip_monitor_status(**args)

    def test_high_score_is_not_confirmed_event(self):
        status = self.status()
        self.assertEqual(status['state'], 'CONTACT SETTLING')
        self.assertTrue(status['sensors'][0]['high'])
        self.assertFalse(status['confirmed_this_grasp'])

    def test_confirmation_and_last_event_survive_current_low_score(self):
        self.guard.initial_clear = True
        self.guard.high_counts = [2, 0]
        self.assertEqual(self.status()['state'], 'CONFIRMING SLIP')
        self.guard.confirmed_event = self.prediction
        self.guard.rearm_sensor = 0
        status = self.status(prediction=(1, (10, 10), (.1, .1)))
        self.assertTrue(status['confirmed_this_grasp'])
        self.assertFalse(status['sensors'][0]['high'])
        self.assertAlmostEqual(status['last_confirmed_age_sec'], .1)
        self.assertFalse(self.status(contact=False)['confirmed_this_grasp'])
        self.assertFalse(self.status(epoch=2)['confirmed_this_grasp'])

    def test_missing_stale_invalid_data_are_not_live_scores(self):
        for prediction in (None, (1, (9, 9), (.9, .1)), (2, (10, 10), (.9, .1)),
                           (1, (10, 10), (float('nan'), .1))):
            status = self.status(prediction=prediction)
            self.assertEqual(status['state'], 'MODEL UNAVAILABLE')
            self.assertIsNone(status['sensors'][0]['score'])
            self.assertFalse(status['sensors'][0]['high'])

    def test_modes_and_read_only(self):
        before = copy.deepcopy(self.guard.__dict__)
        self.assertEqual(self.status(enabled=False)['state'], 'OFF')
        self.assertEqual(self.status(probe=True)['state'], 'MANUAL PROBE')
        self.assertEqual(self.status(contact=False)['state'], 'WAITING FOR CONTACT')
        self.assertEqual(before, self.guard.__dict__)
        self.guard.inhibited = True
        self.assertEqual(self.status()['state'], 'REINFORCEMENT INHIBITED')


if __name__ == '__main__':
    unittest.main()
