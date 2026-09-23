import unittest

from gripper_position_tracking import PositionTracker


class OwnershipTest(unittest.TestCase):
    def test_logged_lever_play_keeps_slip_motion_authorized(self):
        p = self.paused()
        p.release_delta = .03
        for i, value in enumerate((.590, .588, .595, .589), 1):
            p.command(value, i * .02)
            self.assertIsNone(p.tick(i * .02, .02, [[100]*3]*2, [i, i], True))
            self.assertTrue(p.contact)
            self.assertTrue(p.can_issue_motion('reinforcement', i * .02))
            self.assertEqual(p.hold_input, .6)
        p.command(.56, .2)
        self.assertFalse(p.can_issue_motion('reinforcement', .2))
        self.assertIsNotNone(p.tick(.2, .02, [[100]*3]*2, [6, 6], True))
        self.assertFalse(p.contact)

    def test_full_open_bypasses_release_deadband(self):
        p = self.paused()
        p.release_delta = .03
        p.hold_input = .02
        p.command(0, .1)
        self.assertTrue(p.opening_requested(0))
        self.assertFalse(p.can_issue_motion('reinforcement', .1))

    def paused(self):
        p = PositionTracker((2000, 2000), (1520, 2480), (1752, 2246), 2919)
        p.command(.6, 0)
        p.contact = True
        p.hold_input = .6
        return p

    def test_hold_does_not_overwrite_reinforcement(self):
        p = self.paused()
        self.assertEqual(p.motion_phase, 'contact paused')
        self.assertFalse(p.can_issue_motion('tracking', .01))
        self.assertTrue(p.can_issue_motion('reinforcement', .01))
        p.acknowledge_reinforcement((1749, 2249))
        for i in range(1, 5):
            p.command(.7, i * .02)
            self.assertIsNone(p.tick(i * .02, .02, [[100]*3]*2, [i, i], True))
            self.assertFalse(p.stop_requested)
            self.assertEqual(p.target, (1749, 2249))
            self.assertEqual(p.hold_input, .6)
            self.assertTrue(p.contact)
            self.assertEqual(p.motion_phase, 'slip reinforcement')
        p.finish_reinforcement()
        self.assertEqual(p.motion_phase, 'contact paused')
        self.assertEqual(p.target, (1749, 2249))

    def test_open_preempts_reinforcement_without_new_hold(self):
        p = self.paused()
        p.acknowledge_reinforcement((1749, 2249))
        p.command(0, .1)
        self.assertFalse(p.can_issue_motion('reinforcement', .1))
        target = p.tick(.1, .02, [[100]*3]*2, [1, 1], True)
        self.assertIsNotNone(target)
        self.assertGreater(target[0], 1749)
        self.assertLess(target[1], 2249)
        self.assertFalse(p.contact)
        self.assertFalse(p.reinforcement_active)
        self.assertFalse(p.stop_requested)
        self.assertTrue(p.can_issue_motion('tracking', .1))

    def test_pending_hold_stale_input_or_block_cannot_reinforce(self):
        p = self.paused()
        self.assertFalse(p.can_issue_motion('reinforcement', 1))
        p.stop_requested = True
        self.assertFalse(p.can_issue_motion('reinforcement', .1))
        p.stop_requested = False
        p.blocked = True
        self.assertFalse(p.can_issue_motion('reinforcement', .1))


if __name__ == '__main__':
    unittest.main()
