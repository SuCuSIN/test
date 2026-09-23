"""Bounded sensor-change heuristic; this does not measure slip or grip force."""

import math
import statistics


class RegraspGuard:
    def __init__(self):
        self.reset()

    def reset(self):
        self.reference = self.initial = None
        self.range_low = None
        self.response_reference = None
        self.tokens = None
        self.window = []
        self.started = None
        self.count = self.attempts = 0
        self.pending_channel = None
        self.pending_direction = None
        self.cooldown = 0.0
        self.inhibited = False
        self.state = 'waiting for contact'
        self.change_by_sensor = [0.0, 0.0]
        self.direction_by_sensor = ['inside', 'inside']

    def inhibit(self, reason):
        self.inhibited = True
        self.state = reason
        return False

    def response_exceeded(self, values):
        return (self.response_reference is not None and
                any(v - a > 60 for v, a in zip(values, self.response_reference)))

    def note_move(self, values):
        # Capture immediately BEFORE the first reinforcement, not first contact.
        # External pulling may already have raised the signal substantially.
        if self.response_reference is None:
            self.response_reference = list(values)

    def observe(self, now, rows, tokens, margin):
        if self.inhibited:
            return False
        if len(rows) != 2 or len(tokens) != 2 or any(len(row) != 3 for row in rows):
            return self.inhibit('two sensors with three independent groups required')
        if not all(math.isfinite(v) for row in rows for v in row):
            return self.inhibit('invalid or stale sensor; no regrasp until next grasp')
        contact_sensors = [max(row) >= margin for row in rows]
        if not any(contact_sensors):
            if self.initial is None:
                self.window = []
                self.started, self.count = None, 0
                self.state = 'waiting for contact on at least one sensor before reference'
                return False
            return self.inhibit('contact lost on both sensors; no automatic chase')
        previous = self.tokens
        if previous is not None and any(a < b for a, b in zip(tokens, previous)):
            return self.inhibit('sensor sample counter reset')
        if previous is not None and any(a == b for a, b in zip(tokens, previous)):
            return False
        self.tokens = tuple(tokens)
        values = [v for row in rows for v in row]
        if self.response_exceeded(values):
            return self.inhibit('post-regrasp rise cap reached; further reinforcement blocked')
        if now < self.cooldown:
            return False
        if self.attempts >= 3:
            return self.inhibit('three-attempt regrasp limit reached')
        if self.reference is None:
            self.state = 'collecting contact range (0.5s)'
            self.window.append((now, values))
            self.window = [item for item in self.window if now - item[0] <= 0.7]
            if len(self.window) < 5 or now - self.window[0][0] < 0.5:
                return False
            # Freeze each group's central 80% envelope. Normal sensor jitter is
            # allowed; the reference must not follow a later sustained excursion.
            deciles = [statistics.quantiles([item[1][i] for item in self.window],
                                            n=10, method='inclusive') for i in range(6)]
            self.range_low = [values[0] for values in deciles]
            self.reference = [values[8] for values in deciles]
            if self.initial is None:
                self.initial = list(self.reference)
            self.window = []
            self.state = 'range ready; watching upper/lower excursions'
            return False
        signed = [v - upper if v > upper else v - lower if v < lower else 0.0
                  for v, lower, upper in zip(values, self.range_low, self.reference)]
        excess = [abs(v) for v in signed]
        peaks = [max(indexes, key=excess.__getitem__) for indexes in (range(3), range(3, 6))]
        self.change_by_sensor = [excess[i] for i in peaks]
        self.direction_by_sensor = ['above' if signed[i] > 0 else 'below' if signed[i] < 0
                                    else 'inside' for i in peaks]
        # An unloaded sensor's magnetic changes cannot trigger tightening.
        eligible = [value if contact_sensors[i // 3] else 0.0 for i, value in enumerate(excess)]
        channel = (self.pending_channel if self.pending_channel is not None and
                   eligible[self.pending_channel] >= 20 else max(range(6), key=eligible.__getitem__))
        changed = eligible[channel] >= 20
        if not changed:
            self.started, self.count = None, 0
            self.pending_channel = None
            self.pending_direction = None
            self.state = 'range ready; watching upper/lower excursions'
            return False
        direction = 'above' if signed[channel] > 0 else 'below'
        if self.pending_channel != channel or self.pending_direction != direction:
            self.started, self.count = None, 0
            self.pending_channel = channel
            self.pending_direction = direction
        self.state = (f'confirming {direction} range: sensor {channel // 3 + 1}, '
                      f'group {channel % 3 + 1}')
        if self.started is None:
            self.started = now
        self.count += 1
        if self.count < 3 or now - self.started < 0.12:
            return False
        self.attempts += 1
        self.reference = None
        self.range_low = None
        self.pending_channel = None
        self.pending_direction = None
        self.started, self.count = None, 0
        self.cooldown = now + 0.7
        self.state = 'suspected slip: bounded regrasp requested'
        return True


def continuation_error(episode, prediction, now, epoch, contacts, threshold):
    """Authorize a second bounded step, never replay the old slip prediction."""
    if epoch != episode['epoch'] or not 0 <= now - episode['started'] <= 3.0:
        return 'bounded episode expired or calibration changed'
    if prediction is None:
        return 'second step requires fresh model scores'
    model_epoch, stamps, scores = prediction
    if (model_epoch != epoch or len(stamps) != 2 or len(scores) != 2 or len(contacts) != 2
            or any(not math.isfinite(t) or not 0 <= now - t <= .25 for t in stamps)
            or any(not math.isfinite(s) or not 0 <= s <= 1 for s in scores)):
        return 'second step model input invalid or stale'
    if not any(contact and score >= threshold for contact, score in zip(contacts, scores)):
        return 'slip no longer high on a contacting sensor; no second step'
    return None


def bounded_target(measured, previous, origin, opened, closed):
    return bounded_target_check(measured, previous, origin, opened, closed)[0]


def bounded_target_check(measured, previous, origin, opened, closed, step=2,
                         advance_previous=False, tracking_tolerance=3,
                         clamp_to_budget=False):
    if tracking_tolerance not in (3, 6) or (tracking_tolerance == 6 and not advance_previous and step != 6):
        return None, 'unsupported tracking tolerance'
    if step not in (2, 3, 6) or (step == 6 and advance_previous):
        return None, 'unsupported reinforcement step'
    if any(len(row) != 2 for row in (measured, previous, origin, opened, closed)):
        return None, 'two jaw positions required'
    result = []
    for jaw, (value, old, start, a, b) in enumerate(zip(measured, previous, origin, opened, closed), 1):
        direction = 1 if b > a else -1
        if value is None or not all(math.isfinite(v) for v in (value, old, start, a, b)):
            return None, f'jaw {jaw}: position missing or nonfinite'
        if a == b or not all(min(a, b) <= v <= max(a, b) for v in (value, old, start)):
            return None, f'jaw {jaw}: position outside limits'
        # Learned steps allow the six-count stationary error seen in the probe.
        # The worker still requires observed motion before another step.
        if abs(value - old) > tracking_tolerance:
            return None, f'jaw {jaw}: previous target unresolved (error={value - old})'
        base = old if advance_previous else value
        if advance_previous and clamp_to_budget:
            # Advance beyond both the acknowledged goal and measured jaw position.
            base = old if direction * (old - value) >= 0 else value
        goal = base + direction * step
        if clamp_to_budget:
            # Preserve the contact-origin budget even when measured position leads the goal.
            goal = start + direction * min(direction * (goal - start), 6)
            if direction * (goal - value) <= 0:
                return None, f'jaw {jaw}: cumulative travel budget exhausted'
        travel_limit = tracking_tolerance + step
        if advance_previous and not 0 < direction * (goal - value) <= travel_limit:
            return None, f'jaw {jaw}: measured-to-target travel outside 1..{travel_limit} counts'
        if not min(a, b) <= goal <= max(a, b):
            return None, f'jaw {jaw}: next step outside limits'
        if not 0 <= direction * (goal - start) <= 6:
            return None, f'jaw {jaw}: cumulative travel limit'
        if direction * (goal - old) <= 0:
            return None, f'jaw {jaw}: step would reopen or repeat previous target'
        result.append(goal)
    return tuple(result), None
