from ur5e_gello_state_publisher.body_resistance import BodyResistance


class Packet:
    def __init__(self):
        self.registers = {33: 0, 40: 0, 48: 1000, 49: 3}
        self.writes = []
        self.bad_limit = False

    def syncWriteTxOnly(self, port, address, width, params, count):
        assert params[0] == 2
        self.writes.append((address, params[1:]))
        self.registers.update({address + i: v for i, v in enumerate(params[1:])})
        return 0

    def read1ByteTxRx(self, port, servo_id, address):
        return self.registers.get(address, 0), 0, 0

    def read2ByteTxRx(self, port, servo_id, address):
        if self.bad_limit:
            return 1000, 0, 0
        return self.registers[address] + 256 * self.registers[address + 1], 0, 0


def cue():
    packet = Packet()
    resistance = BodyResistance(packet, None)
    resistance.feedback(True, 1)
    resistance.update(2000, 1)
    return packet, resistance


def test_small_lag_and_cap_before_enable():
    packet, resistance = cue()
    resistance.update(2005, 1.02)
    assert resistance.active
    assert packet.registers[42] + 256 * packet.registers[43] == 2003
    assert packet.registers[48] == 10
    enable_index = packet.writes.index((40, [1]))
    assert any(address == 48 for address, _ in packet.writes[:enable_index])


def test_rest_releases_and_reversal_tracks_current():
    packet, resistance = cue()
    resistance.update(2005, 1.02)
    resistance.update(2005, 1.04)
    assert not resistance.active
    assert packet.registers[40] == 0
    resistance.update(2000, 1.06)
    assert packet.registers[42] + 256 * packet.registers[43] == 2002


def test_contact_release_and_stale():
    for contact_age in (0, 1):
        packet, resistance = cue()
        resistance.update(2005, 1.02)
        resistance.feedback(bool(contact_age), 1.02 - contact_age)
        resistance.update(2010, 1.04)
        assert packet.registers[40] == 0


def test_bad_limit_never_enables():
    packet = Packet()
    packet.bad_limit = True
    resistance = BodyResistance(packet, None)
    resistance.feedback(True, 1)
    resistance.update(2000, 1)
    resistance.update(2005, 1.02)
    assert resistance.fault
    assert (40, [1]) not in packet.writes


def test_missing_position_and_gap_and_wrap_latch_fault():
    for raw, now in ((None, 1.04), (2010, 2), (4095, 1.04)):
        packet, resistance = cue()
        resistance.update(2005, 1.02)
        resistance.update(raw, now)
        assert resistance.fault
        assert packet.registers[40] == 0


def test_mode_not_changed():
    packet = Packet()
    packet.registers[33] = 1
    resistance = BodyResistance(packet, None)
    resistance.update(2000, 1)
    assert resistance.fault
    assert all(address != 33 for address, _ in packet.writes)


def test_disable_and_close_keep_low_limit():
    packet, resistance = cue()
    resistance.update(2005, 1.02)
    resistance.update(2010, 1.04, enabled=False)
    resistance.close()
    assert packet.registers[40] == 0
    assert packet.registers[48] == 10


def test_no_contact_no_enable():
    packet = Packet()
    resistance = BodyResistance(packet, None)
    resistance.update(2000, 1)
    resistance.update(2010, 1.02)
    assert (40, [1]) not in packet.writes


def test_bench_returns_before_any_motion_publish():
    from types import SimpleNamespace
    from ur5e_gello_state_publisher.ur5e_gello_publisher import UR5eGelloPublisher

    node = SimpleNamespace(
        control_mode='disabled', body_resistance=None,
        enable_gripper_haptic_feedback=False, body_resistance_bench=True,
        reader=SimpleNamespace(read_joints_and_gripper=lambda: ([0] * 6, 0)))
    # No motion publishers/RTDE interface exist on this stub: touching one fails.
    UR5eGelloPublisher.publish_state(node)
