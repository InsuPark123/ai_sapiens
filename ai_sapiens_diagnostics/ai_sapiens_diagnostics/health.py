# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Versioned app contract; no ROS, device access, or command side effects."""

from collections import deque
from copy import deepcopy
import math
import uuid


def fault_history(fields, elapsed=0.0):
    """Optional observed episodes, independent of current health and clock epochs."""
    keys = ('fault_occurrences', 'fault_clearances',
            'last_fault_started_age_ms', 'last_fault_cleared_age_ms')
    result = {'status': 'unsupported', 'occurrences': None, 'clearances': None,
              'last_started_age_ms': None, 'last_cleared_age_ms': None,
              'scope': 'producer_session'}
    if not any(key in fields for key in keys):
        return result
    try:
        values = []
        for index, key in enumerate(keys):
            raw = fields[key]
            if (not isinstance(raw, str) or len(raw) > 20 or
                    not (raw.isascii() and raw.isdecimal() or index >= 2 and raw == '-1')):
                raise ValueError('invalid history integer')
            number = int(raw)
            if number > (2**64 - 1 if index < 2 else 2**63 - 1):
                raise ValueError('history integer overflow')
            values.append(number)
        raised, cleared, started_age, cleared_age = values
        if ((raised == 0) != (started_age == -1) or
                (cleared == 0) != (cleared_age == -1) or cleared > raised):
            raise ValueError('inconsistent history')
    except (KeyError, ValueError, TypeError):
        result['status'] = 'invalid'
        return result
    result.update(
        status='available', occurrences=str(raised), clearances=str(cleared),
        last_started_age_ms=None if started_age == -1 else started_age + elapsed * 1000,
        last_cleared_age_ms=None if cleared_age == -1 else cleared_age + elapsed * 1000)
    return result


class HealthStore:
    def __init__(self, expected, robot_id='unassigned', producer_timeout=1.0,
                 history_size=64):
        if not math.isfinite(producer_timeout) or producer_timeout <= 0:
            raise ValueError('producer_timeout must be finite and positive')
        if history_size < 1 or len(set(expected)) != len(expected):
            raise ValueError('invalid history size or duplicate expected component')
        self.expected = set(expected)
        self.robot_id = robot_id
        self.timeout = producer_timeout
        self.boot_id = str(uuid.uuid4())
        self.entries = {}
        self.previous = {}
        self.history = deque(maxlen=history_size)
        self.dropped_events = 0
        self.event_id = 0
        self.sequence = 0
        self.rejected_messages = 0

    def ingest(self, name, hardware_id, fields, now):
        if not name.startswith('dhi/'):
            return False
        try:
            if fields['schema_version'] != '1':
                raise ValueError('unsupported schema')
            session = fields['producer_session']
            sequence = int(fields['sequence'])
            success = int(fields['rx_successes'])
            age = float(fields['sample_age_ms']) / 1000.0
            stale = float(fields['stale_after_ms']) / 1000.0
            if not session or sequence < 1 or success < 0:
                raise ValueError('invalid identity or sequence')
            if not math.isfinite(age) or not math.isfinite(stale) or stale <= 0 or age < -0.001:
                raise ValueError('invalid sample age')
            if age < 0 and success > 0:
                raise ValueError('received sample has negative age')
            if fields['health'] not in {'OK', 'WARN', 'ERROR', 'UNKNOWN'}:
                raise ValueError('invalid health')
            if fields['quality'] not in {'FRESH', 'STALE', 'MISSING', 'INVALID'}:
                raise ValueError('invalid quality')
            if fields['health'] == 'OK' and (
                fields['quality'] != 'FRESH' or success == 0 or
                fields.get('fault_collected') != 'true'
            ):
                raise ValueError('unsubstantiated healthy state')
            # Ensure exported counters are well formed before using them for events.
            for key in ('rx_failures', 'tx_failures', 'fault_events', 'capture_drops'):
                if int(fields.get(key, '0')) < 0:
                    raise ValueError('negative counter')
        except (KeyError, ValueError, TypeError):
            self.rejected_messages += 1
            # A malformed producer must not refresh a previously healthy entry.
            old = self.entries.get(name)
            if old is not None:
                old['invalid'] = True
            return False

        old = self.entries.get(name)
        retired = list(old['retired']) if old else []
        same_session = old is not None and old['session'] == session
        if old and (session in retired or (same_session and sequence <= old['sequence'])):
            self.rejected_messages += 1
            return False
        if same_session and success < old['rx_successes']:
            self.rejected_messages += 1
            old['invalid'] = True
            return False
        if old and not same_session:
            retired = (retired + [old['session']])[-8:]
        progress = old['progress'] if same_session and success == old['rx_successes'] else now
        self.entries[name] = {
            'hardware_id': hardware_id, 'fields': dict(fields), 'received': now,
            'session': session, 'sequence': sequence, 'rx_successes': success,
            'progress': progress, 'age': age, 'stale': stale, 'retired': retired,
            'invalid': False,
        }
        return True

    def _component(self, name, now):
        entry = self.entries.get(name)
        if entry is None:
            return {'name': name, 'required': name in self.expected,
                    'health': 'UNKNOWN', 'quality': 'MISSING',
                    'reason': 'PRODUCER_MISSING', 'evidence': {},
                    'fault_history': fault_history({})}
        fields = entry['fields']
        elapsed = max(0.0, now - entry['received'])
        age = None if entry['rx_successes'] == 0 else max(
            0.0, entry['age'] + elapsed, now - entry['progress'])
        health, quality, reason = fields['health'], fields['quality'], fields.get('reason', '')
        if entry['invalid']:
            health, quality, reason = 'UNKNOWN', 'INVALID', 'INVALID_DIAGNOSTIC'
        elif elapsed > self.timeout:
            health, quality, reason = 'UNKNOWN', 'STALE', 'PRODUCER_EXPIRED'
        elif age is not None and age > entry['stale']:
            health, quality, reason = 'UNKNOWN', 'STALE', 'SAMPLE_EXPIRED'
        measurements = {}
        for key, value in fields.items():
            if key.startswith('value.'):
                field = key[6:]
                try:
                    number = float(value)
                    number = number if math.isfinite(number) else None
                except ValueError:
                    number = None
                measurements[field] = {
                    'value': number, 'unit': fields.get('unit.' + field, 'unknown'),
                    'quality': quality,
                }
        return {
            'name': name, 'hardware_id': entry['hardware_id'],
            'required': name in self.expected, 'health': health,
            'quality': quality, 'reason': reason,
            'message': fields.get('message', reason) if reason == fields.get('reason') else reason,
            'bus': fields.get('bus'), 'dxl_id': fields.get('dxl_id'),
            'joint': fields.get('joint'), 'component_type': fields.get('component_type'),
            'producer_session': entry['session'], 'producer_sequence': entry['sequence'],
            'sample_age_ms': None if age is None else age * 1000.0,
            'producer_age_ms': elapsed * 1000.0,
            'source_freshness': fields.get('source_freshness', 'unknown'),
            'measurements': measurements,
            'fault_history': fault_history(fields, elapsed),
            'evidence': {key: value for key, value in fields.items()
                         if not key.startswith(('value.', 'unit.'))},
        }

    def snapshot(self, now):
        self.sequence += 1
        components = [self._component(name, now)
                      for name in sorted(self.expected | self.entries.keys())]
        events = []
        for component in components:
            name = component['name']
            evidence = component['evidence']
            signature = (
                component['health'], component['quality'], component['reason'],
                component.get('producer_session'),
                evidence.get('fault_events', '0'),
                evidence.get('rx_failures', '0'), evidence.get('tx_failures', '0'),
                evidence.get('last_error_code', '0'),
                evidence.get('last_hardware_error', '0'), evidence.get('last_estop_active', '0'),
                component['fault_history']['status'],
                component['fault_history']['occurrences'],
                component['fault_history']['clearances'],
            )
            previous = self.previous.get(name)
            if previous != signature:
                self.event_id += 1
                event = {
                    'schema_version': 1, 'robot_id': self.robot_id,
                    'boot_id': self.boot_id, 'event_id': self.event_id, 'component': name,
                    'health': component['health'], 'quality': component['quality'],
                    'reason': component['reason'], 'previous': previous,
                    'producer_session': component.get('producer_session'),
                    'fault_history': deepcopy(component['fault_history']),
                    'evidence': deepcopy(evidence),
                }
                if len(self.history) == self.history.maxlen:
                    self.dropped_events += 1
                self.history.append(event)
                events.append(event)
                self.previous[name] = signature
        required = [c for c in components if c['required']]
        known_faults = [c['name'] for c in components if c['health'] in ('ERROR', 'WARN')]
        unknown_required = [c['name'] for c in required if c['health'] == 'UNKNOWN']
        health = ('ERROR' if any(c['health'] == 'ERROR' for c in required) else
                  'UNKNOWN' if unknown_required or not required else
                  'WARN' if any(c['health'] == 'WARN' for c in required) else 'OK')
        return {
            'schema_version': 1, 'robot_id': self.robot_id, 'boot_id': self.boot_id,
            'sequence': self.sequence, 'health': health,
            'scope': 'configured_observations_only',
            'known_faults': known_faults, 'unknown_required': unknown_required,
            'components': components, 'recent_events': list(self.history),
            'history_dropped_total': self.dropped_events,
            'rejected_messages': self.rejected_messages,
        }, events
