"""Host-observed displacement intervals, not physical motor onset timestamps."""


class GripperMotionTiming:
    def __init__(self):
        self.samples = {}
        self.anchors = {}
        self.pending = {}

    def command(self, targets, sent_ns, seq):
        for servo_id, target in enumerate(targets, 1):
            pending = self.pending.get(servo_id)
            if pending:
                if pending['direction'] * (target - pending['base']) < 3:
                    self.pending.pop(servo_id, None)
                    self.anchors.pop(servo_id, None)
                continue
            sample = self.samples.get(servo_id)
            anchor = self.anchors.get(servo_id)
            if (sample is None or anchor is None or sent_ns - sample[1] > 350_000_000
                    or sample[1] - anchor[1] < 300_000_000 or abs(target - sample[0]) < 3):
                continue
            self.pending[servo_id] = dict(base=sample[0], start=sent_ns, seq=seq,
                direction=1 if target > sample[0] else -1)

    def observe(self, servo_id, raw, observed_ns):
        previous = self.samples.get(servo_id)
        self.samples[servo_id] = (raw, observed_ns)
        if previous is None or not 0 < observed_ns - previous[1] <= 350_000_000:
            self.pending.pop(servo_id, None)
            self.anchors[servo_id] = (raw, observed_ns)
            return None
        pending = self.pending.get(servo_id)
        if pending:
            if observed_ns - pending['start'] > 2_000_000_000:
                self.pending.pop(servo_id)
            elif pending['direction'] * (raw - pending['base']) >= 3:
                self.pending.pop(servo_id)
                self.anchors[servo_id] = (raw, observed_ns)
                return dict(component='gripper_motion', seq=pending['seq'], servo_id=servo_id,
                    send_pc_ns=pending['start'], observed_lower_pc_ns=previous[1],
                    observed_upper_pc_ns=observed_ns,
                    observation_lower_ms=max(0, (previous[1] - pending['start']) / 1e6),
                    observation_upper_ms=(observed_ns - pending['start']) / 1e6,
                    threshold_raw=3, baseline_raw=pending['base'], actual_raw=raw)
        anchor = self.anchors.get(servo_id)
        if anchor is None or abs(raw - anchor[0]) >= 3:
            self.anchors[servo_id] = (raw, observed_ns)
        return None
