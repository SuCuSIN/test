import json
import unittest
from types import SimpleNamespace

from ur5e_gello_state_publisher.tracking_report_recorder import TrackingReportRecorder


class DiagnosticRecordingTest(unittest.TestCase):
    def test_diagnostics_without_seq_do_not_enter_joint_timings(self):
        recorder = SimpleNamespace(gripper_diagnostic_frames=[], timing_frames=[])
        for component in ('gripper_motor_diagnostic', 'gripper_regrasp_position'):
            frame = {'component': component, 'id': 1, 'actual_raw': 1750}
            TrackingReportRecorder.timing_callback(recorder, SimpleNamespace(data=json.dumps(frame)))
        self.assertEqual(len(recorder.gripper_diagnostic_frames), 2)
        self.assertEqual(recorder.timing_frames, [])


if __name__ == '__main__':
    unittest.main()
