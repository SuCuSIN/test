"""Read-only SMS_STS register decoding; preserve raw data, no force conversion.

Addresses follow the vendored Waveshare SMS_STS.h in waveshare_sts3215_usb_diag.
"""

import json


def parse_diagnostic(line, servo_id, request):
    if not line.startswith('DIAG '):
        return None
    try:
        record = json.loads(line[5:])
        if record['id'] != servo_id or record['request'] != request:
            return None
        raw = record['registers_26_70']
        if record['ok'] is not True or record['status'] != 0:
            return None
        if len(raw) != 45 or any(type(v) is not int or not 0 <= v <= 255 for v in raw):
            return None
        control = record.get('registers_21_25')
        if control is not None and (not isinstance(control, list) or len(control) != 5
                or any(type(v) is not int or not 0 <= v <= 255 for v in control)):
            return None
        # Waveshare ST3215 memory map V3.7, decimal addresses (not hex).
        record['control_settings_available'] = control is not None
        record['control_registers_raw'] = (dict(zip(('21', '22', '23', '24', '25'), control))
                                           if control is not None else None)
        if control is not None:
            record.update(position_p_raw=control[0], position_d_raw=control[1],
                          position_i_raw=control[2], minimum_start_output_raw=control[3] | control[4] << 8)
        def byte(address):
            return raw[address - 26]
        def word(address):
            return byte(address) | byte(address + 1) << 8
        record.update(deadband_cw_raw=byte(26), deadband_ccw_raw=byte(27),
                      mode_raw=byte(33), acceleration_raw=byte(41),
                      goal_time_raw=word(44), goal_speed_raw=word(46),
                      torque_limit_raw=word(48), moving_raw=byte(66), servo_status_raw=byte(65),
                      torque_enable_raw=byte(40), goal_raw=word(42), actual_raw=word(56),
                      speed_raw=word(58), load_raw=word(60), voltage_raw=byte(62),
                      temperature_raw=byte(63), current_raw=word(69))
        load = word(60)
        # The load register is signed PWM duty, not measured grip force.
        record['pwm_signed_raw'] = -(load & ~1024) if load & 1024 else load
        record['pwm_percent'] = record['pwm_signed_raw'] / 10
        return record
    except (ValueError, KeyError, TypeError, IndexError):
        return None
