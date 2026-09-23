import unittest

from load_feedback_signal import LoadFeedbackSignal


class SignalTests(unittest.TestCase):
    def setUp(self):
        self.signal = LoadFeedbackSignal()
        for i in range(121):
            self.signal.update(i * 0.02, [0, 0, -6], [0] * 6, True, False, 0)

    def load(self):
        for i in range(121, 182):
            self.signal.update(i * 0.02, [0, -10, -6], [0] * 6, True, True, 0)

    def test_reference_and_latch(self):
        self.load()
        self.assertAlmostEqual(self.signal.level, 0.5)
        level = self.signal.update(3.64, [500, 0, 0], [1] * 6, False, True, 0)
        self.assertAlmostEqual(level, 0.5)

    def test_release_clears(self):
        self.load()
        self.assertEqual(self.signal.update(3.64, [0] * 3, [0] * 6, True, False, 0), 0)

    def test_stale_clears(self):
        self.load()
        self.assertEqual(self.signal.update(3.64, [0] * 3, [0] * 6, True, True, 1), 0)
        self.assertIsNone(self.signal.reference)

    def test_pose_change_blocks(self):
        self.signal.update(2.42, [0, -20, 0], [1] * 6, True, True, 0)
        self.assertEqual(self.signal.level, 0)
        self.assertEqual(self.signal.state, 'reference pose mismatch')

    def test_no_reference_blocks(self):
        signal = LoadFeedbackSignal()
        self.assertEqual(signal.update(1, [20] * 3, [0] * 6, True, True, 0), 0)

    def test_invalid_clears(self):
        self.load()
        self.assertEqual(self.signal.update(3.64, [float('nan')] * 3,
                                            [0] * 6, True, True, 0), 0)


if __name__ == '__main__':
    unittest.main()
