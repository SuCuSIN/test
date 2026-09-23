import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import bluetooth_anyskin_gripper_shell as shell


class ContactPauseTest(unittest.TestCase):
    def run_close(self, signal):
        clock = SimpleNamespace(now=0.0)
        moves = []

        def sleep(seconds):
            clock.now += seconds

        client = SimpleNamespace(
            read_position=lambda *args, **kwargs: 0,
            sync_move=lambda *args: moves.append((clock.now, args)),
        )
        monitor = SimpleNamespace(
            enabled=True, ports=["sensor1", "sensor2"], ignored_indexes=set(),
            num_mags=5,
            magnet_strengths=lambda: signal(clock.now, len(moves)),
        )
        hold = Mock()
        with patch.multiple(
            shell,
            time=SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep),
            open_raws=lambda config: (0, 0),
            close_raws=lambda config: (100, 100),
            empty_baseline_magnet_strengths=lambda *args: [[0] * 5] * 2,
            classify_material_hint=lambda *args: {"contact_confirmed": True},
            print_material_hint=lambda *args: None,
            hold_gentle_contact_position=hold,
        ):
            result = shell.close_safe(
                client, monitor, {}, speed=280, acc=22,
                confirm_sec=0.10, poll_sec=0.01, timeout_sec=1,
                hold_speed=120, hold_acc=12, print_interval_sec=10,
                min_close_sec=0, baseline_margin=45, confirm_samples=4,
                contact_min_channels=1, contact_min_sensors=2,
                single_channel_strong_delta=220, min_contact_ratio=0,
                steps=4, step_interval_sec=0.02, stop_backoff_raw=0,
                material_soft_delta=80, material_hard_delta=180,
                material_soft_rise_rate=450, material_hard_rise_rate=1000,
                material_rise_window_sec=0.2, allow_unbaselined=True,
            )
        return result, moves, hold

    def test_sustained_contact_does_not_advance_during_confirmation(self):
        result, moves, hold = self.run_close(
            lambda now, count: [[100 if count else 0] * 5] * 2
        )
        self.assertTrue(result["contact_confirmed"])
        self.assertEqual(len(moves), 1)
        hold.assert_called_once()

    def test_short_spike_resumes_without_confirming_contact(self):
        result, moves, hold = self.run_close(
            lambda now, count: [[100 if count and now < 0.04 else 0] * 5] * 2
        )
        self.assertFalse(result.get("contact_confirmed", False))
        self.assertEqual(len(moves), 4)
        self.assertGreaterEqual(moves[1][0], 0.04)
        hold.assert_not_called()

    def test_sensor_loss_during_confirmation_holds(self):
        result, moves, hold = self.run_close(
            lambda now, count: [[float("nan") if now >= 0.03 else (100 if count else 0)] * 5] * 2
        )
        self.assertFalse(result.get("contact_confirmed", False))
        self.assertEqual(len(moves), 1)
        hold.assert_called_once()


if __name__ == "__main__":
    unittest.main()
