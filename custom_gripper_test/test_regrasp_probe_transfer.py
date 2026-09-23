import unittest

from gripper_regrasp import bounded_target_check


class ProbeTransferTests(unittest.TestCase):
    def automatic_target(self, measured, previous=(1771, 2226), origin=(1771, 2226)):
        return bounded_target_check(measured, previous, origin, (2000, 2000), (1520, 2480),
                                    step=3, advance_previous=True, tracking_tolerance=6,
                                    clamp_to_budget=True)

    def test_logged_automatic_goal_equal_to_actual_now_advances(self):
        self.assertEqual(self.automatic_target((1768, 2227)), ((1765, 2230), None))

    def test_automatic_remaining_budget_is_clamped_per_jaw(self):
        self.assertEqual(self.automatic_target((1766, 2231)), ((1765, 2232), None))

    def test_automatic_two_episodes_keep_original_budget(self):
        first, error = self.automatic_target((1771, 2226))
        self.assertIsNone(error)
        self.assertEqual(first, (1768, 2229))
        second, error = self.automatic_target(first, previous=first)
        self.assertIsNone(error)
        self.assertEqual(second, (1765, 2232))
        target, error = self.automatic_target(second, previous=second)
        self.assertIsNone(target)
        self.assertIn('exhausted', error)

    def test_automatic_all_tracking_offsets_respect_bounds(self):
        for left in range(-6, 7):
            for right in range(-6, 7):
                measured = (1771 + left, 2226 + right)
                target, error = self.automatic_target(measured)
                if target is None:
                    self.assertTrue(left == -6 or right == 6)
                    self.assertIn('exhausted', error)
                    continue
                for value, old, goal, direction in zip(measured, (1771, 2226), target, (-1, 1)):
                    self.assertGreater(direction * (goal - value), 0)
                    self.assertGreater(direction * (goal - old), 0)
                    self.assertLessEqual(direction * (goal - old), 6)
                    self.assertLessEqual(direction * (goal - value), 9)

    def test_automatic_rejects_endpoint_and_unresolved_position(self):
        self.assertIsNone(self.automatic_target((1764, 2226))[0])
        self.assertIsNone(self.automatic_target((1521, 2479), (1521, 2479), (1521, 2479))[0])

    def clamped_target(self, measured, previous=(1755, 2243), origin=(1755, 2243)):
        return bounded_target_check(measured, previous, origin, (2000, 2000), (1520, 2480),
                                    step=6, tracking_tolerance=6, clamp_to_budget=True)

    def test_logged_one_count_lead_uses_remaining_five(self):
        self.assertEqual(self.clamped_target((1754, 2244)), ((1749, 2249), None))

    def test_clamped_budget_in_both_directions(self):
        for lead in range(6):
            target, reason = self.clamped_target((1755 - lead, 2243 + lead))
            self.assertIsNone(reason)
            self.assertEqual(target, (1749, 2249))

    def test_clamped_exhausted_budget_never_reopens(self):
        for measured in ((1749, 2249), (1749, 2244)):
            target, reason = self.clamped_target(measured)
            self.assertIsNone(target)
            self.assertIn('exhausted', reason)

    def test_clamped_goal_never_repeats(self):
        target, reason = self.clamped_target((1750, 2248), previous=(1749, 2249))
        self.assertIsNone(target)
        self.assertIn('repeat', reason)

    def test_clamping_preserves_endpoint_and_tracking_checks(self):
        self.assertIsNone(self.clamped_target((1762, 2243))[0])
        self.assertIsNone(self.clamped_target((1523, 2477), (1523, 2477), (1523, 2477))[0])

    def measured_target(self, measured, previous, origin):
        return bounded_target_check(measured, previous, origin, (2000, 2000), (1520, 2480),
                                    step=6, tracking_tolerance=6)

    def test_actual_grasp_first_trial_uses_six(self):
        self.assertEqual(self.measured_target((1754, 2244), (1754, 2244), (1754, 2244)),
                         ((1748, 2250), None))

    def test_actual_grasp_second_trial_uses_six(self):
        self.assertEqual(self.measured_target((1736, 2262), (1736, 2264), (1736, 2264)),
                         ((1730, 2268), None))

    def test_measured_mode_preserves_total_limit(self):
        target, reason = self.measured_target((1750, 2248), (1748, 2250), (1754, 2244))
        self.assertIsNone(target)
        self.assertIn('cumulative', reason)

    def test_measured_mode_never_repeats_goal(self):
        target, reason = self.measured_target((1754, 2244), (1748, 2250), (1754, 2244))
        self.assertIsNone(target)
        self.assertIn('repeat', reason)

    def test_measured_mode_cannot_exceed_endpoint(self):
        self.assertIsNone(self.measured_target((1523, 2477), (1523, 2477), (1523, 2477))[0])

    def target(self, measured, previous=(1760, 2240)):
        return bounded_target_check(measured, previous, (1760, 2240),
                                    (2000, 2000), (1520, 2480), step=3,
                                    advance_previous=True, tracking_tolerance=6)

    def test_recorded_probe_positions(self):
        self.assertEqual(self.target((1763, 2234)), ((1757, 2243), None))

    def test_second_episode_keeps_command_increment(self):
        self.assertEqual(self.target((1761, 2238), (1757, 2243)), ((1754, 2246), None))

    def test_cumulative_cap_unchanged(self):
        target, reason = self.target((1754, 2246), (1754, 2246))
        self.assertIsNone(target)
        self.assertIn('cumulative', reason)

    def test_larger_error_still_blocked(self):
        self.assertIsNone(self.target((1767, 2234))[0])

    def test_no_reopening(self):
        self.assertIsNone(self.target((1756, 2244))[0])


if __name__ == '__main__':
    unittest.main()
