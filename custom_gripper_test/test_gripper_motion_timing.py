from gripper_motion_timing import GripperMotionTiming


def ready():
    timing = GripperMotionTiming()
    for stamp in (0, 200_000_000, 400_000_000):
        timing.observe(1, 2000, stamp)
    timing.command((1990, 2010), 410_000_000, 3)
    return timing


def test_position_feedback_bracket_not_ack():
    timing = ready()
    assert timing.observe(1, 1999, 500_000_000) is None
    event = timing.observe(1, 1995, 600_000_000)
    assert event['observation_lower_ms'] == 90
    assert event['observation_upper_ms'] == 190
    assert event['seq'] == 3
    assert timing.observe(1, 1990, 700_000_000) is None


def test_missing_feedback_and_reversal_do_not_produce_latency():
    timing = ready()
    assert timing.observe(1, 1900, 900_000_000) is None
    timing = ready()
    timing.command((2010, 1990), 450_000_000, 4)
    assert timing.observe(1, 1990, 600_000_000) is None


def test_no_motion_is_not_zero_latency():
    timing = ready()
    assert timing.observe(1, 2000, 600_000_000) is None
