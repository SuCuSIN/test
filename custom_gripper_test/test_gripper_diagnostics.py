import json
import unittest
from unittest.mock import Mock

from gripper_diagnostics import parse_diagnostic
from bluetooth_anyskin_gripper_shell import BluetoothGripperClient


class DiagnosticTest(unittest.TestCase):
    def record(self, **changes):
        raw = [0] * 45
        raw[0:2] = [4, 5]
        raw[40-26] = 1
        raw[42-26:44-26] = [0xd1, 6]  # 1745
        raw[56-26:58-26] = [0xd4, 6]  # 1748
        raw[62-26] = 120
        raw[69-26:71-26] = [30, 0x80]  # Preserve sign bit; no guessed force units.
        result = dict(id=1, request=1, ok=True, status=0, registers_26_70=raw)
        result.update(changes)
        return 'DIAG ' + json.dumps(result)

    def test_register_mapping(self):
        r = parse_diagnostic(self.record(), 1, 1)
        self.assertEqual((r['goal_raw'], r['actual_raw']), (1745, 1748))
        self.assertEqual((r['deadband_cw_raw'], r['deadband_ccw_raw']), (4, 5))
        self.assertEqual(r['torque_enable_raw'], 1)
        self.assertEqual(r['voltage_raw'], 120)
        self.assertEqual(r['current_raw'], 0x801e)

    def test_bad_failed_or_unmatched_frames_rejected(self):
        for line in ('DIAG garbage', 'POSITION id=1 raw=4', self.record(request=2),
                     self.record(id=2), self.record(ok=False), self.record(status=4),
                     self.record(registers_26_70=[0]*44),
                     self.record(registers_26_70=[256]*45)):
            self.assertIsNone(parse_diagnostic(line, 1, 1))

    def test_extended_control_settings_and_old_firmware_compatibility(self):
        r = parse_diagnostic(self.record(registers_21_25=[32, 0, 0, 16, 0]), 1, 1)
        self.assertTrue(r['control_settings_available'])
        self.assertEqual(r['control_registers_raw']['21'], 32)
        self.assertEqual(r['position_p_raw'], 32)
        self.assertEqual(r['position_i_raw'], 0)
        self.assertEqual(r['minimum_start_output_raw'], 16)
        self.assertEqual(r['goal_raw'], 1745)
        old = parse_diagnostic(self.record(), 1, 1)
        self.assertFalse(old['control_settings_available'])

    def test_load_is_signed_pwm_not_force_or_overload(self):
        raw = [0]*45
        raw[60-26:62-26] = [48, 4]
        raw[65-26] = 32
        r = parse_diagnostic(self.record(registers_26_70=raw), 1, 1)
        self.assertEqual(r['load_raw'], 1072)
        self.assertEqual(r['pwm_signed_raw'], -48)
        self.assertEqual(r['pwm_percent'], -4.8)
        self.assertEqual(r['servo_status_raw'], 32)

    def test_invalid_control_settings_rejected(self):
        for values in ([], [0]*4, [0]*6, [256]*5, '12345', [True]*5):
            self.assertIsNone(parse_diagnostic(self.record(registers_21_25=values), 1, 1))

    def test_client_sends_only_diagnostic_read_and_ignores_late_reply(self):
        c = BluetoothGripperClient('unused', 0)
        c.position_callback = Mock()
        def send(command):
            c.lines.put(self.record(request=0))
            c.lines.put(self.record())
        c.send = Mock(side_effect=send)
        r = c.read_diagnostics(1)
        c.send.assert_called_once_with('DIAG 1 1')
        c.position_callback.assert_called_once_with(1, 1748)
        self.assertEqual(r['goal_raw'], 1745)

    def test_old_firmware_does_not_retry_or_move(self):
        c = BluetoothGripperClient('unused', 0)
        c.send = Mock(side_effect=lambda command: c.lines.put('ERROR: unknown command'))
        self.assertIsNone(c.read_diagnostics(1))
        c.send.assert_called_once_with('DIAG 1 1')


if __name__ == '__main__':
    unittest.main()
