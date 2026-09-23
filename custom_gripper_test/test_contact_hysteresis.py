from gripper_position_tracking import PositionTracker


def test_three_fresh_consecutive_samples_and_duration_are_required():
    tracker = PositionTracker((2000, 2000), (1520, 2480), (1800, 2200),
                              2919, margin=20, confirm_sec=.04, confirm_samples=3,
                              required_sensors=1)
    tracker.command(.8, 0)
    for now, token, value in ((0, 1, 25), (.02, 2, 25), (.05, 3, 19),
                              (.06, 4, 25), (.08, 5, 25), (.11, 5, 25)):
        tracker.tick(now, .02, [[value]], [token], True)
        assert not tracker.contact
    assert tracker.tick(.12, .02, [[25]], [6], True) is None
    assert tracker.contact


def test_single_spike_does_not_latch_and_motion_resumes():
    tracker = PositionTracker((2000, 2000), (1520, 2480), (1800, 2200),
                              2919, margin=20, confirm_sec=.04, confirm_samples=3,
                              required_sensors=1)
    tracker.command(.8, 0)
    tracker.tick(0, .02, [[100]], [1], True)
    assert not tracker.contact
    assert tracker.tick(.02, .02, [[0]], [2], True) is not None
    assert not tracker.contact


def test_immediate_contact_latches_without_further_motion_until_operator_opens():
    tracker = PositionTracker((2000, 2000), (1520, 2480), (1800, 2200),
                              2919, margin=20, confirm_sec=0, confirm_samples=1,
                              required_sensors=1)
    tracker.command(0.5, 0)
    assert tracker.tick(0, .02, [[21]], [1], True) is None
    assert tracker.contact
    # Updated actual position can lie beyond the commanded ratio.
    tracker.acknowledge((1730, 2270))
    tracker.command(0.5, .02)
    assert tracker.tick(.02, .02, [[0]], [2], True) is None
    assert tracker.contact
    tracker.command(0.9, .04)
    assert tracker.tick(.04, .02, [[0]], [3], True) is None
    assert tracker.contact
    tracker.command(0.85, .06)
    assert tracker.tick(.06, .02, [[0]], [4], True) is None
    assert tracker.contact
    tracker.command(0.45, .08)
    target = tracker.tick(.08, .02, [[0]], [5], True)
    assert not tracker.contact
    assert target[0] >= 1730 and target[1] <= 2270


def test_lower_margin_confirms_sustained_signal_but_not_a_spike():
    def run(values):
        tracker = PositionTracker((2000, 2000), (1520, 2480), (1800, 2200),
                                  120, margin=35, required_sensors=1)
        tracker.command(0.8, 0)
        for i, value in enumerate(values):
            tracker.tick(i * .03, .03, [[value]], [i], True)
        return tracker.contact
    assert run([38] * 5)
    assert not run([38, 0, 0, 0, 0])
    assert not run([30] * 5)


def test_threshold_dips_reset_confirmation():
    tracker = PositionTracker((2000, 2000), (1520, 2480), (1800, 2200),
                              120, required_sensors=1)
    tracker.command(0.8, 0)
    for i, value in enumerate((46, 42, 46, 40, 48)):
        tracker.tick(i * .03, .03, [[value]], [i], True)
    assert not tracker.contact
    tracker = PositionTracker((2000, 2000), (1520, 2480), (1800, 2200),
                              120, required_sensors=1)
    tracker.command(0.8, 0)
    for i, value in enumerate((80, 10, 10, 10, 10)):
        tracker.tick(i * .03, .03, [[value]], [i], True)
    assert not tracker.contact
