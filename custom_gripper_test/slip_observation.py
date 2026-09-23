"""Independent slip telemetry. Never grants motor motion permission."""
import math


class SlipObservation:
    def __init__(self):
        self.total = 0
        self.grasp_count = 0
        self.active = False
        self.contact = False
        self.epoch = None
        self.since = None
        self.last = None
        self.tokens = None
        self.high = [0, 0]
        self.low = 0
        self.last_confirmed = None
        self.snapshot = {}

    def update(self, now, prediction, epoch, contact, rows, margin):
        transitions = []
        boundary = contact != self.contact or epoch != self.epoch
        if boundary:
            if self.active:
                transitions.append('interrupted')
            self.active = False
            self.high = [0, 0]
            self.low = 0
            self.tokens = None
            self.grasp_count = 0
            self.last_confirmed = None
            self.since = now if contact else None
        self.contact, self.epoch = contact, epoch
        valid = (prediction is not None and prediction[0] == epoch
                 and len(prediction[1]) == len(prediction[2]) == 2
                 and all(math.isfinite(t) and 0 <= now-t <= .25 for t in prediction[1])
                 and all(math.isfinite(s) and 0 <= s <= 1 for s in prediction[2]))
        contacts = [len(r) == 3 and all(math.isfinite(v) for v in r)
                    and max(r) >= margin*.5 for r in rows]
        if len(contacts) != 2:
            contacts = [False, False]
        gap = self.last is not None and now-self.last > .35
        if not valid or gap:
            if self.active:
                transitions.append('interrupted')
            self.active = False
            self.high = [0, 0]
            self.low = 0
            self.tokens = None
        state = 'MODEL UNAVAILABLE' if not valid else 'WAITING FOR CONTACT'
        if valid and contact:
            stamps, scores = prediction[1:]
            fresh = [self.tokens is None or t > self.tokens[i] for i, t in enumerate(stamps)]
            if any(fresh):
                self.last = now
            self.tokens = tuple(stamps)
            if min(stamps) <= self.since+.3:
                state = 'CONTACT SETTLING'
                self.high = [0, 0]
            else:
                if not self.active:
                    for i in range(2):
                        if fresh[i]:
                            self.high[i] = self.high[i]+1 if scores[i] >= .8 and contacts[i] else 0
                    if max(self.high) >= 3:
                        self.active = True
                        self.grasp_count += 1
                        self.total += 1
                        self.last_confirmed = now
                        self.low = 0
                        transitions.append('started')
                elif all(fresh):
                    self.low = self.low+1 if all(s <= .3 for s in scores) else 0
                    if self.low >= 3:
                        self.active = False
                        self.high = [0, 0]
                        transitions.append('ended')
                state = 'SLIP ACTIVE' if self.active else 'CONFIRMING SLIP' if any(self.high) else 'NO CURRENT SLIP'
        self.snapshot = dict(state=state, active=self.active, count=self.grasp_count,
                             total_count=self.total, high_counts=list(self.high),
                             clear_samples=self.low, contact_mask=contacts,
                             last_confirmed_pc_s=self.last_confirmed, observed_pc_s=now,
                             transitions=transitions)
        return self.snapshot
