"""Experimental relative-load cue, not a mass estimate or actuator command."""

import math
import statistics


class LoadFeedbackSignal:
    def __init__(self):
        self.reference = None
        self.reference_q = None
        self.empty = []
        self.loaded = []
        self.level = 0.0
        self.state = 'waiting for empty stationary reference'
        self.last_time = None

    def invalidate(self, reason):
        self.__init__()
        self.state = reason
        return 0.0

    def update(self, now, force, q, stationary, contact, contact_age):
        values = [now, contact_age, *force, *q]
        if len(force) != 3 or len(q) != 6 or not all(math.isfinite(v) for v in values):
            return self.invalidate('invalid data')
        if contact_age < 0 or contact_age > 0.5:
            return self.invalidate('contact heartbeat missing')
        if self.last_time is not None and not 0 < now - self.last_time <= 0.5:
            return self.invalidate('force data gap or clock reset')
        self.last_time = now
        if not contact:
            self.level = 0.0
            self.loaded = []
            if not stationary:
                self.empty = []
                self.reference = None
                self.state = 'waiting for stationary empty reference'
                return 0.0
            self.empty.append((now, list(force), list(q)))
            self.empty = [item for item in self.empty if now - item[0] <= 2.5]
            if now - self.empty[0][0] >= 2.0:
                if any(max(item[2][i] for item in self.empty) -
                       min(item[2][i] for item in self.empty) > 0.02 for i in range(6)):
                    self.reference = None
                    self.state = 'empty pose not settled'
                    return 0.0
                self.reference = [statistics.median(item[1][i] for item in self.empty)
                                  for i in range(3)]
                self.reference_q = list(q)
                self.state = 'reference ready'
            return 0.0
        self.empty = []
        if self.reference is None:
            self.state = 'no empty reference; release object first'
            return 0.0
        if self.state == 'relative cue latched':
            return self.level
        if max(abs(a - b) for a, b in zip(q, self.reference_q)) > 0.05:
            self.loaded = []
            self.state = 'reference pose mismatch'
            return 0.0
        if not stationary:
            self.loaded = []
            self.state = 'waiting for load to settle'
            return 0.0
        self.loaded.append((now, list(force)))
        self.state = 'collecting stationary loaded samples'
        if now - self.loaded[0][0] >= 1.0:
            med = [statistics.median(item[1][i] for item in self.loaded) for i in range(3)]
            # All-axis change is an empirical load cue, NOT gravity or kilograms.
            delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(med, self.reference)))
            self.level = min(1.0, max(0.0, (delta - 5.0) / 10.0))
            self.state = 'relative cue latched'
        return self.level
