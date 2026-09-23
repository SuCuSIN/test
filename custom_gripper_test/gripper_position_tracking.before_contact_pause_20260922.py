"""Hardware-independent position tracking and contact gating."""

import math


class PositionTracker:
    def __init__(self, open_raws, close_raws, initial_raws, speed,
                 margin=45.0, confirm_sec=0.10, confirm_samples=4,
                 required_sensors=2, command_timeout=0.5):
        self.open = tuple(open_raws)
        self.closed = tuple(close_raws)
        self.target = tuple(initial_raws)
        self.speed = speed
        self.margin = margin
        self.confirm_sec = confirm_sec
        self.confirm_samples = confirm_samples
        self.required = required_sensors
        self.command_timeout = command_timeout
        self.desired = None
        self.received = None
        self.contact = False
        self.blocked = False
        self.state = "waiting for controller"
        self.timers = {}
        self.last_tokens = {}
        self.hold_input = None
        self.offset = 0.0
        self.opening_release = False

    def command(self, ratio, now):
        if not math.isfinite(ratio):
            return
        self.desired = min(1.0, max(0.0, ratio))
        self.received = now

    def ratio(self):
        ratios = [(v - a) / (b - a) for v, a, b in
                  zip(self.target, self.open, self.closed) if b != a]
        return min(1.0, max(0.0, sum(ratios) / len(ratios)))

    def tick(self, now, dt, channels, sample_tokens, valid_baseline):
        if self.desired is None:
            return None
        current = self.ratio()
        released = self.hold_input is not None and self.desired < self.hold_input - 0.01
        # A measured-pose update must not look like an operator opening command.
        if self.contact and not released and self.desired > 0.003:
            # Keep the contact anchor fixed: recoil after pulling harder is not OPEN.
            self.state = "contact hold"
            return None
        if released:
            # Release on a lever reversal even if it was pulled beyond contact.
            self.offset = max(0.0, self.desired - max(0.0, current - 0.01))
            self.hold_input = None
        if self.desired <= 0.003:
            self.offset = 0.0
        requested = max(0.0, self.desired - self.offset)
        if now - self.received > self.command_timeout:
            self.blocked = True
            self.hold_input = self.desired
            self.timers.clear()
            self.state = "controller timeout; open to rearm"
            return None
        # Opening intent clears the latch even when no travel remains.
        # Keep it clear at the opening target until a new closing request.
        if released or self.desired <= 0.003 or requested < current - 0.003:
            self.opening_release = True
        elif requested > current + 0.003:
            self.opening_release = False
        opening = requested < current - 0.003 or self.opening_release
        if opening:
            self.contact = self.blocked = False
            self.hold_input = None
            self.timers.clear()
            self.state = "opening"
        else:
            if self.contact or self.blocked:
                self.hold_input = max(self.desired, self.hold_input or 0.0)
                self.state = "contact hold" if self.contact else "blocked; open to rearm"
                return None
            if not valid_baseline:
                self.state = "empty-close calibration required"
                return None
            if not channels or any(not row or any(not math.isfinite(v) for v in row)
                                   for row in channels):
                self.blocked = True
                self.hold_input = self.desired
                self.state = "sensor lost; open to rearm"
                return None
            pending = False
            mature = set()
            for sensor, row in enumerate(channels):
                fresh = self.last_tokens.get(sensor) != sample_tokens[sensor]
                self.last_tokens[sensor] = sample_tokens[sensor]
                for channel, delta in enumerate(row):
                    key = (sensor, channel)
                    if delta < self.margin:
                        self.timers.pop(key, None)
                        continue
                    pending = True
                    start, count = self.timers.get(key, (now, 0))
                    count += int(fresh)
                    self.timers[key] = (start, count)
                    if fresh and now - start >= self.confirm_sec and count >= self.confirm_samples:
                        mature.add(sensor)
            if len(mature) >= self.required:
                self.contact = True
                self.hold_input = self.desired
                self.state = "contact hold"
                return None
            if pending:
                self.hold_input = max(self.desired, self.hold_input or 0.0)
                self.state = "confirming contact; target held"
                return None
            self.state = "tracking"
            self.hold_input = None
        if self.desired <= 0.003 and current <= 0.003:
            self.offset = 0.0
            self.blocked = self.contact = False
        desired = tuple(round(a + requested * (b - a))
                        for a, b in zip(self.open, self.closed))
        distance = max(abs(b - a) for a, b in zip(self.target, desired))
        if not distance:
            return None
        # Bound target lead even when Bluetooth ACKs or scheduling are delayed.
        step = max(1, min(8, self.speed * min(dt, 0.02)))
        fraction = min(1.0, step / distance)
        target = tuple(round(a + fraction * (b - a))
                       for a, b in zip(self.target, desired))
        return target if target != self.target else None

    def acknowledge(self, target):
        self.target = tuple(target)
