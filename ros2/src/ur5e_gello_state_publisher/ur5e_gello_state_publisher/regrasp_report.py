"""Read-only episode summaries. Encoder displacement is not grip force."""
import html
import math


def slip_intervals(frames, origin):
    """Observed model episodes, not ground-truth slip or motor permission.

    Enter at >= .8; end after three distinct <= .3 samples. Never bridge
    missing telemetry, calibration changes, or a released contact.
    """
    intervals = []
    active = None
    last = None
    epoch = None
    tokens = None
    low = 0
    low_start = None
    for frame in sorted(frames, key=lambda f: f['observed_pc_ns']):
        now = frame['observed_pc_ns'] / 1e9
        prediction = frame.get('slip_model_prediction')
        valid = (isinstance(prediction, (list, tuple)) and len(prediction) == 3
                 and len(prediction[1]) == len(prediction[2]) == 2
                 and all(isinstance(t, (int, float)) and math.isfinite(t)
                         and 0 <= now - t <= .25 for t in prediction[1])
                 and all(isinstance(s, (int, float)) and math.isfinite(s)
                         and 0 <= s <= 1 for s in prediction[2]))
        gap = last is not None and now - last > .35
        changed = valid and epoch is not None and epoch != prediction[0]
        if active is not None and (not valid or gap or changed or not frame.get('contact')):
            intervals.append(dict(start_sec=active-origin, end_sec=last-origin,
                                  end_reason='contact released' if not frame.get('contact') else 'observation interrupted'))
            active = None
            low = 0
        if not valid or not frame.get('contact'):
            tokens = None
            continue
        if changed or gap:
            tokens = None
        epoch, stamps, scores = prediction
        fresh = tokens is None or all(t > old for t, old in zip(stamps, tokens))
        if not fresh:
            continue
        tokens = stamps
        last = now
        contacts = frame.get('slip_contact_mask', [True, True])
        if active is None and any(s >= .8 and c for s, c in zip(scores, contacts)):
            active = now
            low = 0
        if active is not None:
            if all(s <= .3 for s in scores):
                if low == 0:
                    low_start = now
                low += 1
                if low >= 3:
                    intervals.append(dict(start_sec=active-origin, end_sec=low_start-origin,
                                          end_reason='model cleared'))
                    active = None
                    low = 0
            else:
                low = 0
    if active is not None:
        intervals.append(dict(start_sec=active-origin, end_sec=last-origin,
                              end_reason='end not observed'))
    return intervals


def episodes(diagnostics, commands, origin):
    result = []
    for event in sorted((f for f in diagnostics
                         if f.get('component') == 'gripper_regrasp_event'),
                        key=lambda f: f['observed_pc_ns']):
        key = event['episode_id']
        moves = sorted((f for f in commands if f.get('regrasp')
                        and f.get('episode_id') == key), key=lambda f: f['send_start_ns'])
        positions = sorted((f for f in diagnostics
                            if f.get('component') == 'gripper_regrasp_position'
                            and f.get('episode_id') == key),
                           key=lambda f: f['observed_pc_ns'])
        row = dict(episode_id=key, trigger=event['trigger'],
                   time_sec=event['observed_pc_ns'] / 1e9 - origin,
                   command_times_sec=[f['send_start_ns'] / 1e9 - origin for f in moves],
                   command_stages=len(moves))
        for index in range(2):
            motor = index + 1
            samples = [f for f in positions if f['servo_id'] == motor]
            row[f'id{motor}_commanded_closing_raw'] = (
                event['closing_directions'][index] *
                (moves[-1]['target_raw'][index] - event['previous_target_raw'][index])
                if moves else None)
            row[f'id{motor}_observed_closing_raw'] = (
                samples[-1]['closing_delta_raw'] if samples else None)
            row[f'id{motor}_observation_time_sec'] = (
                samples[-1]['observed_pc_ns'] / 1e9 - origin if samples else None)
        row['observation'] = ('not commanded' if not moves else
                              'encoder observations available' if positions else
                              'ACK recorded; displacement unavailable')
        result.append(row)
    return result


def table(rows):
    def value(v):
        return 'Unavailable' if v is None else html.escape(str(v))
    body = []
    for index, row in enumerate(rows, 1):
        cells = [index, f"{row['time_sec']:.3f}", row['trigger'], row['command_stages'],
                 row['id1_commanded_closing_raw'], row['id1_observed_closing_raw'],
                 row['id2_commanded_closing_raw'], row['id2_observed_closing_raw'],
                 row['observation']]
        body.append('<tr>' + ''.join(f'<td>{value(v)}</td>' for v in cells) + '</tr>')
    return ('<section><h2>Slip and reinforcement episodes</h2>'
            '<p>Purple: confirmed slip; blue: reinforcement command. Each episode can contain multiple command stages. '
            'Commanded amounts are final goal minus pre-episode goal. Observed amounts are last sampled encoder '
            'position minus pre-episode measured position, positive toward closing. Units: raw counts, not mm or force. '
            'Missing feedback is not zero movement. Encoder observations alone do not certify successful reinforcement.</p>'
            '<p><a href="gripper_reinforcement.csv">Download episode measurements</a></p>'
            '<table><tr><th>Event</th><th>Time (s)</th><th>Trigger</th><th>Stages</th>'
            '<th>ID1 commanded</th><th>ID1 observed</th><th>ID2 commanded</th><th>ID2 observed</th>'
            '<th>Evidence</th></tr>' + ''.join(body) + '</table>'
            + ('<p>No recorded episode events. Older recordings may not include this telemetry.</p>' if not rows else '')
            + '</section>')
