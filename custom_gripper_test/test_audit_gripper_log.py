import json
import unittest

from audit_gripper_log import audit_lines


class AuditTests(unittest.TestCase):
    def record(self, goal=1725):
        raw = [0] * 45
        def word(address, value):
            raw[address-26:address-24] = [value & 255, value >> 8]
        raw[40-26] = 1
        raw[41-26] = 22
        word(42, goal)
        word(46, 2919)
        word(48, 1000)
        word(56, 1734)
        return dict(id=1, request=1, ok=True, status=0, registers_26_70=raw,
                    phase='after', component='gripper_motor_diagnostic',
                    commanded_target_raw=[1725, 2273])

    def test_existing_registers_decode_without_hardware(self):
        report = audit_lines(['[INFO] Motor DIAG: ' + json.dumps(self.record())])
        self.assertEqual(report['after_goal_mismatches'], 0)
        self.assertEqual(report['settings']['1']['goal_speed_raw'], [2919])
        self.assertEqual(report['settings']['1']['torque_limit_raw'], [1000])
        self.assertEqual(report['settings']['1']['mode_raw'], [0])
        self.assertEqual(report['settings']['1']['acceleration_raw'], [22])

    def test_overwrite_detected_in_jsonl(self):
        report = audit_lines([json.dumps(self.record(goal=1731))])
        self.assertEqual(report['after_goal_mismatches'], 1)

    def test_invalid_diagnostic_is_not_success(self):
        row = self.record()
        row['status'] = 4
        report = audit_lines(['Motor DIAG: bad', json.dumps(row)])
        self.assertEqual(report['valid_records'], 0)
        self.assertEqual(report['rejected_records'], 2)


if __name__ == '__main__':
    unittest.main()
