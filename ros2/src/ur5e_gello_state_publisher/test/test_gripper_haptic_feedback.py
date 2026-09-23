from types import SimpleNamespace
import threading

from ur5e_gello_state_publisher.ur5e_gello_publisher import (
    STS3215UR5eReader,
    UR5eGelloPublisher,
)


class FakePacketHandler:
    def __init__(self):
        self.calls = []

    def syncWriteTxOnly(self, port, address, length, values, param_length):
        assert param_length == length + 1
        assert values[0] == 7
        if length == 1:
            self.calls.append(("write1", port, values[0], address, values[1]))
        else:
            self.calls.append(("write", port, values[0], address, length, list(values[1:])))
        return 0

    def write1ByteTxRx(self, port, servo_id, address, value):
        self.calls.append(("write1", port, servo_id, address, value))
        return 0, 0

    def writeTxRx(self, port, servo_id, address, length, values):
        self.calls.append(
            ("write", port, servo_id, address, length, list(values))
        )
        return 0, 0

    def getTxRxResult(self, result):
        return f"result={result}"

    def read1ByteTxRx(self, port, servo_id, address):
        self.calls.append(("read1", port, servo_id, address))
        return getattr(self, "readback", (1, 0, 0))

    def readTx(self, port, servo_id, address, length):
        self.calls.append(('read1', port, servo_id, address))
        return 0

    def rxPacket(self, port):
        if getattr(self, 'packets', None):
            return self.packets.pop(0), 0
        value, result, error = getattr(self, 'readback', (1, 0, 0))
        return [255, 255, 7, 3, error, value, 0], result


def make_reader():
    reader = object.__new__(STS3215UR5eReader)
    reader.port_handler = SimpleNamespace(
        ser=SimpleNamespace(reset_input_buffer=lambda: None),
        setPacketTimeoutMillis=lambda ms: None, is_using=False)
    reader.packet_handler = FakePacketHandler()
    reader.gripper_servo_id = 7
    reader.torque_enable_addr = 40
    reader.acceleration_addr = 41
    return reader


def test_hold_targets_only_gripper_servo_at_current_position():
    reader = make_reader()

    ok, detail = reader.hold_gripper_at_raw(0x0ABC, 0x0123, 20)

    assert ok is True
    assert detail == "ok"
    assert reader.packet_handler.calls == [
        ("write1", reader.port_handler, 7, 40, 0),
        (
            "write",
            reader.port_handler,
            7,
            41,
            7,
            [20, 0xBC, 0x0A, 0, 0, 0x23, 0x01],
        ),
        ("write1", reader.port_handler, 7, 40, 1),
        ("read1", reader.port_handler, 7, 40),
    ]


def test_hold_readback_failure_releases_torque_and_reports_failure():
    for readback in ((0, 0, 0), (1, -1, 0), (1, 0, 4)):
        reader = make_reader()
        reader.packet_handler.readback = readback
        ok, detail = reader.hold_gripper_at_raw(3700, 120, 20)
        assert not ok
        assert "readback failed" in detail
        assert reader.packet_handler.calls[-1] == (
            "write1", reader.port_handler, 7, 40, 0)


def test_release_disables_only_gripper_servo_torque():
    reader = make_reader()

    ok, detail = reader.release_gripper_hold()

    assert ok is True
    assert detail == "ok"
    assert reader.packet_handler.calls == [
        ("write1", reader.port_handler, 7, 40, 0)
    ]


def test_torque_read_rejects_delayed_position_packet():
    reader = make_reader()
    reader.packet_handler.packets = [[255, 255, 7, 4, 0, 73, 14, 0],
                                     [255, 255, 6, 3, 0, 1, 0]]
    assert reader._read_gripper_torque_enabled() == (1, 0, 0)
    assert not reader.port_handler.is_using


def test_feedback_callback_never_accesses_servo_bus():
    state = SimpleNamespace(haptic_mailbox_lock=threading.Lock(),
                            haptic_pending_clear=False,
                            get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)))
    UR5eGelloPublisher.gripper_haptic_feedback_callback(state, SimpleNamespace(data=True))
    assert state.haptic_mailbox[0] is True
    UR5eGelloPublisher.gripper_haptic_feedback_callback(state, SimpleNamespace(data=False))
    assert state.haptic_pending_clear


def test_mailbox_does_not_replay_stale_contact():
    applied = []
    state = SimpleNamespace(haptic_mailbox_lock=threading.Lock(),
                            haptic_mailbox=(True, FakeTime(8)), haptic_pending_clear=False,
                            gripper_haptic_feedback_timeout_sec=1,
                            get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)),
                            _apply_gripper_haptic_contact=applied.append)
    UR5eGelloPublisher.process_gripper_haptic_feedback(state)
    assert applied == [False]
    assert state.gripper_haptic_last_feedback_time.nanoseconds == 8_000_000_000


def test_mailbox_preserves_clear_before_new_contact():
    applied = []
    state = SimpleNamespace(haptic_mailbox_lock=threading.Lock(),
                            haptic_mailbox=(True, FakeTime(9.9)), haptic_pending_clear=True,
                            gripper_haptic_feedback_timeout_sec=1,
                            get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)),
                            _apply_gripper_haptic_contact=applied.append)
    UR5eGelloPublisher.process_gripper_haptic_feedback(state)
    assert applied == [False, True]
    assert not state.haptic_pending_clear


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return FakeTime((self.nanoseconds - other.nanoseconds) * 1e-9)


def test_contact_hold_persists_but_opening_and_stale_feedback_release():
    releases = []
    state = SimpleNamespace(
        gripper_haptic_hold_active=True,
        gripper_haptic_last_feedback_time=FakeTime(9.9),
        gripper_haptic_feedback_timeout_sec=1.0,
        gripper_haptic_hold_started_time=FakeTime(0),
        gripper_haptic_hold_duration_sec=0,
        gripper_haptic_hold_normalized=0.5,
        gripper_haptic_release_delta=0.015,
        reader=SimpleNamespace(latest_gripper_raw=0.5),
        gripper_raw_to_normalized=lambda raw: raw,
        get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(10)),
        release_gripper_haptic_hold=lambda reason, block_rearm=False: releases.append(reason))
    UR5eGelloPublisher.update_gripper_haptic_hold(state)
    assert releases == []
    state.reader.latest_gripper_raw = 0.52
    UR5eGelloPublisher.update_gripper_haptic_hold(state)
    assert 'toward open' in releases.pop()
    state.reader.latest_gripper_raw = 0.5
    state.gripper_haptic_last_feedback_time = FakeTime(8)
    UR5eGelloPublisher.update_gripper_haptic_hold(state)
    assert 'heartbeat timed out' in releases.pop()


def test_measured_controller_endpoints_cover_full_range():
    state = SimpleNamespace(gripper_min_raw=3425, gripper_max_raw=3924,
                            gripper_wrap_open_raw=700, gripper_encoder_ticks=4096)
    normalize = lambda raw: UR5eGelloPublisher.gripper_raw_to_normalized(state, raw)
    assert normalize(3425) == 0.0
    assert normalize(3924) == 1.0
    assert normalize(3400) == 0.0
    assert normalize(3950) == 1.0
    assert 0.49 < normalize(3674) < 0.51


def test_haptic_hold_releases_after_feedback_pulse_duration():
    releases = []
    now = FakeTime(3.0)
    state = SimpleNamespace(
        gripper_haptic_hold_active=True,
        gripper_haptic_last_feedback_time=FakeTime(2.8),
        gripper_haptic_feedback_timeout_sec=1.0,
        gripper_haptic_hold_started_time=FakeTime(0.0),
        gripper_haptic_hold_duration_sec=3.0,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        release_gripper_haptic_hold=lambda reason, block_rearm=False: releases.append(
            (reason, block_rearm)
        ),
    )

    UR5eGelloPublisher.update_gripper_haptic_hold(state)

    assert releases == [("3.0s feedback pulse completed", True)]
