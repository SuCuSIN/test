import unittest

from ur3_load_probe import sample, summarize


class Receiver:
    def getTimestamp(self):
        return 10.0

    def getActualTCPForce(self):
        return [0, 0, -4.9, 0, 0, 0]

    def getActualQ(self):
        return [0] * 6

    def getActualQd(self):
        return [0] * 6


class LoadProbeTests(unittest.TestCase):
    def test_static_force_is_not_reported_as_mass(self):
        result = summarize([sample(Receiver())])
        self.assertEqual(result['fz']['median'], -4.9)
        self.assertNotIn('mass_kg', result)

    def test_motion_excluded(self):
        receiver = Receiver()
        receiver.getActualQd = lambda: [0.1] * 6
        self.assertEqual(summarize([sample(receiver)])['stationary_samples'], 0)

    def test_inconsistent_snapshot_excluded(self):
        receiver = Receiver()
        stamps = iter([10, 11])
        receiver.getTimestamp = lambda: next(stamps)
        self.assertIsNone(sample(receiver))

    def test_nonfinite_rejected(self):
        receiver = Receiver()
        receiver.getActualTCPForce = lambda: [float('nan')] * 6
        with self.assertRaises(ValueError):
            sample(receiver)

    def test_empty_summary(self):
        self.assertIsNone(summarize([])['fz']['median'])


if __name__ == '__main__':
    unittest.main()
