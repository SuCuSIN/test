"""Opt-in, output-limited position-lag cue. Not calibrated torque control.

The host cannot guarantee release after cable loss or process termination.
Leave the low SRAM limit in place on exit; never restore full power here.
"""

import math


class BodyResistance:
    def __init__(self, packet, port, servo_id=2):
        if servo_id not in (2, 3):
            raise ValueError('Only shoulder (2) or elbow (3) may be selected')
        self.packet, self.port, self.servo_id = packet, port, servo_id
        self.limit = 10  # 1% register setting, NOT a measured torque percentage.
        self.ready = self.active = False
        self.fault = ''
        self.previous = None
        self.previous_time = None
        self.contact = False
        self.contact_time = None
        self.last_release_retry = None

    def write(self, address, values):
        params = [self.servo_id, *values]
        result = self.packet.syncWriteTxOnly(
            self.port, address, len(values), params, len(params))
        if result != 0:
            raise RuntimeError(f'write register {address}: communication={result}')

    def read(self, address, width=1):
        method = self.packet.read1ByteTxRx if width == 1 else self.packet.read2ByteTxRx
        value, result, error = method(self.port, self.servo_id, address)
        if result or error:
            raise RuntimeError(f'read register {address}: communication={result}, error={error}')
        return value

    def initialize(self):
        if self.read(33) != 0:
            raise RuntimeError('Position mode required; mode was NOT changed')
        if self.read(40) != 0:
            raise RuntimeError('Servo must start with torque OFF')
        self.write(48, [self.limit, 0])
        if self.read(48, 2) != self.limit:
            raise RuntimeError('Output-limit readback mismatch')
        self.ready = True

    def release(self, verify=False):
        self.write(40, [0])
        if verify and self.read(40) != 0:
            raise RuntimeError('Torque-OFF readback mismatch')
        self.active = False

    def feedback(self, contact, now):
        self.contact = bool(contact)
        self.contact_time = now

    def fail(self, detail):
        self.fault = detail
        try:
            self.release(verify=True)
        except Exception as exc:
            self.fault += f'; RELEASE NOT CONFIRMED: {exc}'

    def update(self, raw, now, enabled=True):
        if self.fault:
            if self.last_release_retry is None or now - self.last_release_retry >= 0.5:
                self.last_release_retry = now
                try:
                    self.release(verify=True)
                except Exception:
                    pass
            return
        try:
            if not enabled:
                if self.active:
                    self.release(verify=True)
                self.previous = None
                self.previous_time = None
                return
            if not self.ready:
                self.initialize()
            if raw is None or not math.isfinite(now) or not 0 <= raw <= 4095:
                raise RuntimeError('Missing or invalid fresh position')
            previous, previous_time = self.previous, self.previous_time
            self.previous, self.previous_time = raw, now
            fresh_contact = (self.contact and self.contact_time is not None
                             and 0 <= now - self.contact_time <= 0.35)
            if previous_time is not None and not 0 < now - previous_time <= 0.15:
                raise RuntimeError('Controller update gap; restart resistance feature')
            delta = 0 if previous is None else raw - previous
            if abs(delta) > 100:
                raise RuntimeError('Position jump or encoder wrap; resistance inhibited')
            if not fresh_contact or abs(delta) < 2 or not 4 <= raw <= 4091:
                if self.active:
                    self.release(verify=True)
                return
            # At most two counts of lag, recomputed from every fresh observation.
            # This limits return travel, but is not guaranteed passive damping.
            target = raw - (2 if delta > 0 else -2)
            if not self.active:
                self.write(48, [self.limit, 0])
                if self.read(48, 2) != self.limit:
                    raise RuntimeError('Output-limit readback mismatch')
            self.write(41, [20, target & 255, target >> 8, 0, 0, 100, 0,
                            self.limit, 0])
            if not self.active:
                self.write(40, [1])
                self.active = True
                if self.read(40) != 1:
                    raise RuntimeError('Torque-ON readback mismatch')
        except Exception as exc:
            self.fail(str(exc))

    def close(self):
        if self.ready or self.active:
            try:
                self.release(verify=True)
            except Exception as exc:
                self.fault = f'RELEASE NOT CONFIRMED: {exc}'
