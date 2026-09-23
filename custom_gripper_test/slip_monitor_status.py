"""Read-only presentation of the model and its reinforcement guard."""

import math


def slip_monitor_status(guard, prediction, error, epoch, now, contact, enabled, probe):
    threshold = getattr(guard, 'SLIP_THRESHOLD', .8)
    valid = (prediction is not None and prediction[0] == epoch
             and len(prediction[1]) == len(prediction[2]) == 2
             and all(math.isfinite(t) and 0 <= now - t <= .25 for t in prediction[1])
             and all(math.isfinite(s) and 0 <= s <= 1 for s in prediction[2]))
    counts = list(getattr(guard, 'high_counts', [0, 0]))
    contacts = list(getattr(guard, 'contact_present', [False, False]))
    event = getattr(guard, 'confirmed_event', None)
    confirmed = event is not None and event[0] == epoch and contact
    if not enabled:
        state = 'OFF'
    elif probe:
        state = 'MANUAL PROBE'
    elif not valid:
        state = 'MODEL UNAVAILABLE'
    elif not contact:
        state = 'WAITING FOR CONTACT'
    elif getattr(guard, 'inhibited', False):
        state = 'REINFORCEMENT INHIBITED'
    elif not getattr(guard, 'initial_clear', False):
        state = 'CONTACT SETTLING'
    elif getattr(guard, 'rearm_sensor', None) is not None:
        state = 'WAITING FOR SLIP TO CLEAR'
    elif any(counts):
        state = 'CONFIRMING SLIP'
    else:
        state = 'WATCHING FOR SLIP'
    return dict(state=state, reason=error or getattr(guard, 'state', 'not initialized'),
                threshold=threshold, required_samples=3, fresh=bool(valid),
                clear_samples=getattr(guard, 'clear_count', 0),
                settling_remaining_sec=(max(0, getattr(guard, 'INITIAL_SETTLE_SEC', .3) -
                    (now - guard.contact_since if getattr(guard, 'contact_since', None) is not None else 0))
                    if contact and not getattr(guard, 'initial_clear', False) else 0),
                waiting_for_rearm=getattr(guard, 'rearm_sensor', None) is not None,
                confirmed_this_grasp=bool(confirmed),
                last_confirmed_age_sec=max(0, now - max(event[1])) if confirmed else None,
                sensors=[dict(score=prediction[2][i] if valid else None,
                              high=bool(valid and prediction[2][i] >= threshold),
                              count=counts[i] if valid and contact and not probe and enabled else 0,
                              contact=contacts[i] if contact else False)
                         for i in range(2)])
