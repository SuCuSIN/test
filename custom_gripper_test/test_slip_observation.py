import unittest
from slip_observation import SlipObservation


class ObservationTests(unittest.TestCase):
    def test_two_episodes_without_motor_guard(self):
        observer = SlipObservation()
        def sample(t, score, contact=True):
            return observer.update(t, (1, (t, t), (score, score)), 1,
                                   contact, [[30]*3]*2, 20)
        sample(0, .9)
        for t in (.31, .36, .41):
            value = sample(t, .9)
        self.assertTrue(value['active'])
        self.assertEqual(value['total_count'], 1)
        for t in (.46, .51, .56):
            self.assertEqual(sample(t, .99)['total_count'], 1)
        for t in (.61, .66, .71):
            value = sample(t, .01)
        self.assertFalse(value['active'])
        self.assertEqual(value['transitions'], ['ended'])
        for t in (.76, .81, .86):
            value = sample(t, .95)
        self.assertEqual(value['total_count'], 2)
        self.assertEqual(value['count'], 2)
        self.assertEqual(sample(.9, .01, False)['count'], 0)
        self.assertEqual(observer.total, 2)

    def test_duplicate_and_stale_do_not_confirm(self):
        observer = SlipObservation()
        observer.update(0, (1, (0, 0), (.9, .9)), 1, True, [[30]*3]*2, 20)
        for now in (.31, .32, .33):
            observer.update(now, (1, (.31, .31), (.9, .9)), 1, True, [[30]*3]*2, 20)
        self.assertEqual(observer.total, 0)
        value = observer.update(1, (1, (.31, .31), (.9, .9)), 1, True, [[30]*3]*2, 20)
        self.assertEqual(value['state'], 'MODEL UNAVAILABLE')
        self.assertEqual(value['high_counts'], [0, 0])


if __name__ == '__main__':
    unittest.main()
