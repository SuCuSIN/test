"""Offline motor register audit. Never connects to or writes to hardware."""
import argparse
import json
from pathlib import Path

from gripper_diagnostics import parse_diagnostic


def audit_lines(lines):
    records = []
    rejected = 0
    for line in lines:
        if 'Motor DIAG: ' in line:
            payload = line.split('Motor DIAG: ', 1)[1]
        elif line.lstrip().startswith('{'):
            payload = line.strip()
        else:
            continue
        try:
            item = json.loads(payload)
        except ValueError:
            rejected += 1
            continue
        if item.get('component') != 'gripper_motor_diagnostic' and 'Motor DIAG: ' not in line:
            continue
        if not all(key in item for key in ('id', 'request')):
            rejected += 1
            continue
        row = parse_diagnostic('DIAG ' + json.dumps(item), item['id'], item['request'])
        if row is None or row['id'] not in (1, 2):
            rejected += 1
            continue
        target = row.get('commanded_target_raw')
        row['goal_matches_command'] = (row['goal_raw'] == target[row['id'] - 1]
                                       if isinstance(target, list) and len(target) == 2 else None)
        records.append(row)
    after = [row for row in records if row.get('phase') == 'after']
    fields = ('mode_raw', 'torque_enable_raw', 'torque_limit_raw', 'acceleration_raw',
              'goal_speed_raw', 'status', 'servo_status_raw')
    return dict(valid_records=len(records), rejected_records=rejected,
                after_records=len(after),
                after_goal_mismatches=sum(row['goal_matches_command'] is False for row in after),
                after_goal_unchecked=sum(row['goal_matches_command'] is None for row in after),
                settings={str(i): {field: sorted({row[field] for row in records if row['id'] == i})
                                    for field in fields} for i in (1, 2)},
                observations=[{key: row.get(key) for key in
                              ('phase', 'id', 'goal_raw', 'actual_raw', 'load_raw', 'pwm_percent', 'current_raw',
                               'position_p_raw', 'position_d_raw', 'position_i_raw', 'minimum_start_output_raw',
                               'voltage_raw', 'goal_matches_command')} for row in records],
                note='PWM is motor duty, not grip force. Sampled agreement cannot exclude brief overwrites.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    with args.log.open(encoding='utf-8-sig') as source:
        print(json.dumps(audit_lines(source), indent=2))


if __name__ == '__main__':
    main()
