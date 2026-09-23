import json
import tempfile
import unittest
from pathlib import Path

from label_anyskin_slip import make_event, validate_recording


class LabelTests(unittest.TestCase):
    def test_records_signed_direction_as_label_not_xyz(self):
        event = make_event('down', 10, 13, 'pose_1', 1, True)
        self.assertEqual(event['label'], 'down')
        self.assertEqual(event['start_pc_monotonic_s'], 10)
        self.assertIn('operator view', event['direction_frame'])

    def test_interrupted_label_invalid_for_training(self):
        self.assertFalse(make_event('press', 10, 11, 'pose_1', 1, False)['completed'])

    def test_bad_label_or_time_rejected(self):
        with self.assertRaises(ValueError):
            make_event('unknown', 10, 11, 'pose_1', 1, True)
        with self.assertRaises(ValueError):
            make_event('down', 11, 10, 'pose_1', 1, True)

    def test_live_schema_and_ended_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / 'metadata.json').write_text(json.dumps({'schema': 'anyskin_xyz_v1'}))
            (folder / 'raw_xyz.csv').write_text('sensor_index\n')
            validate_recording(folder)
            (folder / 'summary.json').write_text('{}')
            with self.assertRaises(ValueError):
                validate_recording(folder)


if __name__ == '__main__':
    unittest.main()
