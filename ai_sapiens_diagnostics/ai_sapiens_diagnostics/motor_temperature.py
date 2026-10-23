# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Bounded K1 temperature display contract; no device access or thermal policy."""
import math
import time

MOTORS = {i: component for component, ids in (
    ('k1_right_arm', range(1, 6)), ('k1_left_arm', range(11, 16)),
    ('k1_left_leg', range(21, 27)), ('k1_right_leg', [*range(31, 37), 41])) for i in ids}
INTERFACES = ('Present Temperature', 'Present Temperature (Motor)')


class MotorTemperatures:
    def __init__(self, robot_id, timeout=0.6):
        if not robot_id or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('robot identity and positive timeout required')
        self.robot_id, self.timeout = robot_id, timeout
        self.samples, self.sources, self.blocked = {}, {}, {}
        self.dynamic_stamp = -1

    def ingest_dynamic(self, rows, stamp, now=None):
        now = time.monotonic() if now is None else now
        try:
            if len(rows) > len(MOTORS) or stamp <= self.dynamic_stamp:
                return False
            values = {}
            for name, interfaces, readings in rows:
                if not name.startswith('dxl') or not name[3:].isascii() or not name[3:].isdigit():
                    raise ValueError('unknown motor')
                identity = int(name[3:])
                if identity not in MOTORS or identity in values or len(interfaces) != len(readings):
                    raise ValueError('invalid temperature row')
                if len(interfaces) != 2 or set(interfaces) != set(INTERFACES):
                    raise ValueError('temperature pair required')
                if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 255 for v in readings):
                    raise ValueError('temperature outside register range')
                fields = dict(zip(interfaces, readings))
                values[identity] = (fields[INTERFACES[0]], fields[INTERFACES[1]], now)
        except (TypeError, ValueError, OverflowError):
            self.samples = {}
            return False
        self.dynamic_stamp = stamp
        self.samples = values
        return True

    def ingest_source(self, name, fields, now=None):
        now = time.monotonic() if now is None else now
        name = name.removeprefix('dhi/')
        parts = name.split('/')
        if len(parts) != 4 or parts[1] != 'dxl' or parts[2] != parts[3]:
            return False
        try:
            identity = int(parts[2])
            if MOTORS.get(identity) != parts[0]:
                return False
            session = fields['producer_session']
            sequence, count = int(fields['publication_sequence']), int(fields['sample_count'])
            sample_age = float(fields['sample_age_ms']) / 1000
            if (not session or len(session) > 128 or sequence < 0 or count < 0 or
                    not math.isfinite(sample_age) or sample_age < 0 or fields['diagnostics_schema'] != '2'):
                raise ValueError('invalid source metadata')
            previous = self.sources.get(identity)
            if previous and previous['session'] == session and sequence <= previous['sequence']:
                return False
            same_session = previous is not None and previous['session'] == session
            progress = same_session and count > previous['count']
            valid = (progress and fields.get('sample_fresh') == 'true' and
                     fields.get('sample_received') == 'true' and fields.get('read_dhi_result') == '0' and
                     sample_age <= self.timeout)
            self.sources[identity] = dict(session=session, sequence=sequence, count=count,
                                         received=now, sample_age=sample_age, valid=valid)
            if not valid:
                self.blocked[identity] = now
            return True
        except (KeyError, TypeError, ValueError, OverflowError):
            if 'identity' in locals() and identity in MOTORS:
                self.sources.pop(identity, None)
                self.blocked[identity] = now
            return False

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        rows = []
        for identity, component in MOTORS.items():
            sample, source = self.samples.get(identity), self.sources.get(identity)
            fresh = bool(sample and source and source['valid'] and
                         0 <= now-sample[2] <= self.timeout and
                         0 <= now-source['received']+source['sample_age'] <= self.timeout and
                         sample[2] > self.blocked.get(identity, -math.inf))
            fields = dict(robot_id=self.robot_id, dxl_id=str(identity), unit='degC',
                          quality='FRESH' if fresh else 'STALE', thermal_limit_evaluated='false',
                          observation_scope='cached_state_and_dhi_sample_freshness')
            if sample:
                fields.update(inverter_temperature_c=str(sample[0]), motor_temperature_c=str(sample[1]))
            rows.append(dict(name=f'temperature/{component}/dxl/{identity}',
                             hardware_id=f'{component}:{identity}', level=0 if fresh else 3,
                             message='Temperature sample; thermal thresholds not evaluated' if fresh else
                                     'Temperature source freshness not confirmed', fields=fields))
        return rows
