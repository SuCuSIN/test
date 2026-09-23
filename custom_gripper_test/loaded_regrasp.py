"""Bounded position-error ramp for loaded servos, not a force controller.

Advance only after reading back the previous goal. Output thresholds below
block further advances; they are NOT hardware current/PWM limits.
"""

import math

STEP_RAW = 3
EPISODE_TRAVEL_RAW = 12
GRASP_TRAVEL_RAW = 24
SETTLE_SEC = 0.4
DEADLINE_SEC = 2.5
OBSERVATION_GRACE_SEC = 0.75
MAX_PWM_RAW = 150
MAX_CURRENT_RAW = 100


def feedback_pending(records, now, after_ns):
    """Missing/old replies need another read, not another movement."""
    return any(r is None or not math.isfinite(r.get('observed_pc_ns', 0))
               or r.get('observed_pc_ns', 0) <= after_ns
               or not 0 <= now - r.get('observed_pc_ns', 0) / 1e9 <= .35
               for r in records)


def signed_magnitude(value, sign_bit):
    sign = 1 << sign_bit
    return -(value & ~sign) if value & sign else value


def feedback_error(records, previous, now, after_ns=0):
    if len(records) != 2:
        return 'both motor diagnostic replies required'
    for index, (record, goal) in enumerate(zip(records, previous), 1):
        prefix = f'jaw {index}: '
        if record is None or record.get('id') != index or record.get('ok') is not True:
            return prefix + 'motor diagnostic unavailable'
        stamp = record.get('observed_pc_ns', 0)
        if not math.isfinite(stamp) or not 0 <= now - stamp / 1e9 <= .35 or stamp <= after_ns:
            return prefix + 'motor diagnostic stale or predates command'
        if record.get('status') != 0 or record.get('servo_status_raw', 0) != 0:
            return prefix + 'motor fault reported'
        if record.get('torque_enable_raw') != 1 or record.get('mode_raw') != 0:
            return prefix + 'position control not enabled'
        if record.get('goal_raw') != goal:
            return prefix + 'goal readback differs from acknowledged target'
        load, current = record.get('load_raw'), record.get('current_raw')
        if (type(load) is not int or not 0 <= load <= 2047 or
                type(current) is not int or not 0 <= current <= 65535):
            return prefix + 'invalid motor output feedback'
        if abs(signed_magnitude(load, 10)) >= MAX_PWM_RAW:
            return prefix + 'PWM observation ceiling reached; no further advance'
        if abs(signed_magnitude(current, 15)) >= MAX_CURRENT_RAW:
            return prefix + 'current observation ceiling reached; no further advance'
        if record.get('torque_limit_raw', 0) <= 0:
            return prefix + 'zero torque limit'
    return None


def next_target(measured, previous, origin, episode_origin, opened, closed,
                settled=(False, False)):
    rows = (measured, previous, origin, episode_origin, opened, closed, settled)
    if any(len(row) != 2 for row in rows):
        return None, 'two jaw values required'
    result = []
    for i, (value, old, start, episode, a, b, done) in enumerate(zip(*rows), 1):
        if not all(type(v) is int for v in (value, old, start, episode, a, b)):
            return None, f'jaw {i}: invalid position'
        if a == b or not all(min(a, b) <= v <= max(a, b) for v in (value, old, start, episode)):
            return None, f'jaw {i}: position outside configured limits'
        direction = 1 if b > a else -1
        if abs(value - old) > EPISODE_TRAVEL_RAW:
            return None, f'jaw {i}: previous goal error exceeds loaded ramp limit'
        if done:
            result.append(old)
            continue
        base = max(direction * old, direction * value)
        goal = direction * min(base + STEP_RAW, direction * episode + EPISODE_TRAVEL_RAW,
                               direction * start + GRASP_TRAVEL_RAW, direction * b)
        if direction * (goal - old) <= 0 or direction * (goal - value) <= 0:
            return None, f'jaw {i}: loaded reinforcement travel budget exhausted'
        result.append(goal)
    target = tuple(result)
    return (target, None) if target != tuple(previous) else (None, 'both jaws already responded')


def episode_error(episode, prediction, now, epoch):
    if epoch != episode['epoch'] or not 0 <= now - episode['started'] <= DEADLINE_SEC:
        return 'loaded episode expired or calibration changed'
    if prediction is None:
        return 'loaded episode requires fresh model input'
    model_epoch, stamps, scores = prediction
    if (model_epoch != epoch or len(stamps) != 2 or len(scores) != 2 or
            any(not math.isfinite(t) or not 0 <= now - t <= .25 for t in stamps) or
            any(not math.isfinite(s) or not 0 <= s <= 1 for s in scores)):
        return 'loaded episode model input invalid or stale'
    # The confirmed event owns this short episode. A fourth high prediction is
    # not required; live contact, sensor rise and opening are rechecked by caller.
    return None
