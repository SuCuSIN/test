import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from bluetooth_anyskin_gripper_shell import RosPositionGripperController


class RegraspTraceTest(unittest.TestCase):
    def controller(self):
        controller = object.__new__(RosPositionGripperController)
        controller.tracker = SimpleNamespace(contact=True, open=(2000, 2000), closed=(1520, 2480),
                                             finish_reinforcement=Mock())
        controller.regrasp_position_trace = dict(before=(1800, 2200), target=(1798, 2202),
                                                ack_ns=1_000_000_000, attempt=1)
        controller.String = SimpleNamespace
        controller.timing_publisher = Mock()
        controller.node = Mock()
        controller.regrasp = Mock()
        return controller

    def test_signed_encoder_change_not_ack_claim(self):
        controller = self.controller()
        controller._record_regrasp_position(1, 1798, 1_200_000_000)
        controller._record_regrasp_position(2, 2200, 1_300_000_000)
        records = [json.loads(call.args[0].data)
                   for call in controller.timing_publisher.publish.call_args_list]
        self.assertEqual([r['closing_delta_raw'] for r in records], [2, 0])
        self.assertEqual(records[0]['after_ack_ms'], 200)

    def test_release_and_expiry_do_not_attribute_motion(self):
        for contact, stamp in ((False, 1_200_000_000), (True, 3_100_000_000)):
            controller = self.controller()
            controller.tracker.contact = contact
            controller._record_regrasp_position(1, 1798, stamp)
            if contact:
                self.assertFalse(controller._regrasp_motion_ready(stamp / 1e9))
                controller.regrasp.inhibit.assert_called_once()
            controller.timing_publisher.publish.assert_not_called()
            self.assertIsNone(controller.regrasp_position_trace)

    def test_requires_two_observations_on_each_jaw(self):
        controller = self.controller()
        controller._record_regrasp_position(1, 1797, 1_100_000_000)
        controller._record_regrasp_position(2, 2203, 1_200_000_000)
        self.assertFalse(controller._regrasp_motion_ready(1.25))
        controller._record_regrasp_position(1, 1797, 1_300_000_000)
        controller._record_regrasp_position(2, 2203, 1_400_000_000)
        self.assertTrue(controller._regrasp_motion_ready(1.45))
        controller.regrasp.inhibit.assert_not_called()

    def test_no_movement_blocks_further_reinforcement(self):
        controller = self.controller()
        controller._record_regrasp_position(1, 1800, 1_200_000_000)
        controller._record_regrasp_position(2, 2200, 1_300_000_000)
        self.assertFalse(controller._regrasp_motion_ready(3.01))
        controller.regrasp.inhibit.assert_called_once()


if __name__ == '__main__':
    unittest.main()
