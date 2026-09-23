import unittest

from gripper_regrasp import RegraspGuard, bounded_target, bounded_target_check


class RegraspTests(unittest.TestCase):
    def ready(self):
        guard = RegraspGuard()
        for i in range(31):
            self.assertFalse(guard.observe(i * .02, [[40] * 3] * 2, [i, i], 20))
        self.assertIsNotNone(guard.reference)
        return guard

    def test_sustained_two_sensors_only(self):
        guard = self.ready()
        outputs = [guard.observe(i * .02, [[65, 40, 40]] * 2, [i, i], 20)
                   for i in range(31, 42)]
        self.assertEqual(sum(outputs), 1)
        self.assertEqual(guard.attempts, 1)

    def test_single_spike(self):
        guard = self.ready()
        self.assertFalse(guard.observe(.62, [[65] * 3] * 2, [31, 31], 20))
        self.assertFalse(guard.observe(.64, [[40] * 3] * 2, [32, 32], 20))
        self.assertEqual(guard.count, 0)

    def test_duplicates_do_not_confirm_or_reset(self):
        guard = self.ready()
        guard.observe(.62, [[65] * 3] * 2, [31, 31], 20)
        for i in range(10):
            self.assertFalse(guard.observe(.63 + i * .01, [[65] * 3] * 2, [31, 31], 20))
        self.assertEqual(guard.count, 1)

    def test_one_sensor_sustained_rise_with_both_in_contact(self):
        guard = self.ready()
        outputs = [guard.observe(i * .02, [[65] * 3, [40] * 3], [i, i], 20)
                   for i in range(31, 42)]
        self.assertEqual(sum(outputs), 1)

    def test_lost_contact_and_bad_signal_inhibit(self):
        for rows in ([[0] * 3] * 2, [[float('nan')] * 3] * 2):
            guard = self.ready()
            self.assertFalse(guard.observe(.62, rows, [31, 31], 20))
            self.assertTrue(guard.inhibited)

    def test_large_pre_move_rise_can_trigger(self):
        guard = self.ready()
        outputs = [guard.observe(i * .02, [[140, 40, 40], [40] * 3], [i, i], 20)
                   for i in range(31, 42)]
        self.assertEqual(sum(outputs), 1)
        self.assertFalse(guard.inhibited)

    def test_post_move_response_cap_even_during_cooldown(self):
        guard = self.ready()
        guard.note_move([140, 40, 40, 40, 40, 40])
        guard.cooldown = 10
        # The response cap must be checked even while the normal trigger is cooling down.
        self.assertFalse(guard.observe(.62, [[201, 40, 40], [40] * 3], [31, 31], 20))
        self.assertTrue(guard.inhibited)

    def test_response_cap_not_ratchet_up_on_repeated_moves(self):
        guard = self.ready()
        guard.note_move([140] * 6)
        guard.note_move([190] * 6)
        self.assertEqual(guard.response_reference, [140] * 6)
        self.assertTrue(guard.response_exceeded([201] * 6))

    def test_counter_reset_inhibits(self):
        guard = self.ready()
        guard.observe(.62, [[40] * 3] * 2, [0, 0], 20)
        self.assertTrue(guard.inhibited)

    def test_either_sensor_alone_can_arm_and_trigger(self):
        for active in (0, 1):
            guard = RegraspGuard()
            rows = [[0] * 3, [0] * 3]
            rows[active] = [40] * 3
            for i in range(31):
                self.assertFalse(guard.observe(i * .02, rows, [i, i], 20))
            self.assertIsNotNone(guard.reference)
            rows[active] = [65] * 3
            outputs = [guard.observe(i * .02, rows, [i, i], 20) for i in range(31, 42)]
            self.assertEqual(sum(outputs), 1)

    def test_unloaded_sensor_changes_do_not_trigger(self):
        guard = RegraspGuard()
        for i in range(31):
            guard.observe(i * .02, [[40] * 3, [0] * 3], [i, i], 20)
        for i in range(31, 50):
            self.assertFalse(guard.observe(i * .02, [[40] * 3, [-40] * 3], [i, i], 20))
        self.assertFalse(guard.inhibited)

    def test_losing_one_contact_does_not_latch_off(self):
        guard = self.ready()
        for i in range(31, 40):
            self.assertFalse(guard.observe(i * .02, [[40] * 3, [0] * 3], [i, i], 20))
        self.assertFalse(guard.inhibited)

    def test_missing_second_stream_still_inhibits(self):
        guard = self.ready()
        guard.observe(.62, [[65] * 3, [float('nan')] * 3], [31, 31], 20)
        self.assertTrue(guard.inhibited)

    def test_attempt_limit(self):
        guard = self.ready()
        guard.attempts = 3
        self.assertFalse(guard.observe(.62, [[65] * 3] * 2, [31, 31], 20))
        self.assertTrue(guard.inhibited)

    def test_noisy_range_arms_without_exact_stability(self):
        guard = RegraspGuard()
        for i in range(50):
            guard.observe(i * .02, [[40 + (i % 2) * 20] * 3] * 2, [i, i], 20)
        self.assertEqual(guard.range_low, [40] * 6)
        self.assertEqual(guard.reference, [60] * 6)
        self.assertEqual(guard.attempts, 0)
        outputs = [guard.observe(i * .02, [[85, 50, 50], [50] * 3], [i, i], 20)
                   for i in range(50, 62)]
        self.assertEqual(sum(outputs), 1)

    def test_sustained_decrease_triggers_while_contact_remains(self):
        guard = self.ready()
        outputs = [guard.observe(i * .02, [[15, 40, 40], [40] * 3], [i, i], 20)
                   for i in range(31, 42)]
        self.assertEqual(sum(outputs), 1)
        self.assertEqual(guard.direction_by_sensor[0], 'below')

    def test_alternating_rise_and_fall_do_not_accumulate(self):
        guard = self.ready()
        for i in range(31, 60):
            self.assertFalse(guard.observe(i * .02,
                [[65 if i % 2 else 15, 40, 40], [40] * 3], [i, i], 20))

    def test_downward_spike_recovers_without_regrasp(self):
        guard = self.ready()
        guard.observe(.62, [[15, 40, 40], [40] * 3], [31, 31], 20)
        self.assertFalse(guard.observe(.64, [[40] * 3] * 2, [32, 32], 20))
        self.assertEqual(guard.count, 0)

    def test_inside_noisy_range_does_not_trigger(self):
        guard = RegraspGuard()
        for i in range(31):
            guard.observe(i * .02, [[40 + (i % 2) * 20] * 3] * 2, [i, i], 20)
        for i in range(31, 60):
            self.assertFalse(guard.observe(i * .02, [[40 + (i % 2) * 20] * 3] * 2,
                                           [i, i], 20))

    def test_alternating_channels_do_not_combine_confirmation(self):
        guard = self.ready()
        for i in range(31, 60):
            row = [40] * 3
            row[i % 2] = 65
            self.assertFalse(guard.observe(i * .02, [row, [40] * 3], [i, i], 20))

    def test_reference_does_not_follow_rise(self):
        guard = self.ready()
        for i in range(31, 40):
            guard.observe(i * .02, [[55] * 3] * 2, [i, i], 20)
        self.assertEqual(guard.reference, [40] * 6)

    def test_target_two_counts_and_cumulative_cap(self):
        self.assertEqual(bounded_target((1800, 2200), (1800, 2200), (1800, 2200),
                                        (2000, 2000), (1520, 2480)), (1798, 2202))
        self.assertIsNone(bounded_target((1794, 2206), (1794, 2206), (1800, 2200),
                                        (2000, 2000), (1520, 2480)))

    def test_no_opening_or_unresolved_motion_or_limit(self):
        for measured, previous in (((1803, 2197), (1800, 2200)),
                                   ((1810, 2190), (1800, 2200)),
                                   ((1520, 2480), (1520, 2480))):
            self.assertIsNone(bounded_target(measured, previous, previous,
                                            (2000, 2000), (1520, 2480)))

    def test_repeat_target_is_not_reinforcement(self):
        target, reason = bounded_target_check((1802, 2198), (1800, 2200),
                                              (1800, 2200), (2000, 2000), (1520, 2480))
        self.assertIsNone(target)
        self.assertIn('repeat previous target', reason)

    def test_learned_step_keeps_six_count_total_limit(self):
        origin = (1800, 2200)
        previous = origin
        for expected in ((1797, 2203), (1794, 2206)):
            target, reason = bounded_target_check(previous, previous, origin,
                                                  (2000, 2000), (1520, 2480), step=3)
            self.assertEqual(target, expected)
            self.assertIsNone(reason)
            previous = target
        target, reason = bounded_target_check(previous, previous, origin,
                                              (2000, 2000), (1520, 2480), step=3)
        self.assertIsNone(target)
        self.assertIn('cumulative', reason)

    def test_missing_position_diagnostic(self):
        target, reason = bounded_target_check((None, 2200), (1800, 2200),
                                              (1800, 2200), (2000, 2000), (1520, 2480))
        self.assertIsNone(target)
        self.assertIn('jaw 1: position missing', reason)

    def test_recorded_slip_advances_both_command_targets(self):
        target, reason = bounded_target_check((1742, 2255), (1742, 2258),
                                              (1742, 2258), (2000, 2000), (1520, 2480),
                                              step=3, advance_previous=True)
        self.assertEqual(target, (1739, 2261))
        self.assertIsNone(reason)

    def test_command_relative_step_cannot_chase_large_position_error(self):
        for measured in ((1742, 2254), (1739, 2261)):
            target, reason = bounded_target_check(measured, (1742, 2258), (1742, 2258),
                                                  (2000, 2000), (1520, 2480), step=3,
                                                  advance_previous=True)
            self.assertIsNone(target)
            self.assertIsNotNone(reason)

    def test_command_relative_steps_stop_at_total_limit(self):
        origin = (1742, 2258)
        previous = origin
        for expected in ((1739, 2261), (1736, 2264)):
            target, reason = bounded_target_check(previous, previous, origin,
                                                  (2000, 2000), (1520, 2480), step=3,
                                                  advance_previous=True)
            self.assertEqual(target, expected)
            previous = target
        target, reason = bounded_target_check(previous, previous, origin,
                                              (2000, 2000), (1520, 2480), step=3,
                                              advance_previous=True)
        self.assertIsNone(target)
        self.assertIn('cumulative', reason)


if __name__ == '__main__':
    unittest.main()
