# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Bridge the original DHI SPSC reporter and startup observer to the app contract.

Only local receipt intervals are measured here. A producer's steady-clock epoch
is never compared with this process's clock. Startup records are history only.
"""
from copy import deepcopy
import re

from .health import HealthStore


DEVICE = re.compile(r'^(.+)/dxl/([0-9]+)/([0-9]+)$')
SDK_RESULTS = {0: 'COMM_SUCCESS', -1000: 'PORT_BUSY', -1001: 'TX_FAIL',
               -1002: 'RX_FAIL', -2000: 'TX_ERROR', -3000: 'RX_WAITING',
               -3001: 'RX_TIMEOUT', -3002: 'RX_CORRUPT', -9000: 'NOT_AVAILABLE'}


def failure_evidence(fields, bus):
    """Use an explicit device origin or the exact paired bus publication."""
    reported = fields.get('failure_origin', '')
    if reported not in ('', 'runtime', 'startup'):
        return 'unknown', 'invalid_device_origin'
    paired = ''
    if (bus and bus.get('diagnostics_schema') == '2' and
            bus.get('phase') == fields.get('phase') == 'FAILED' and
            bus.get('producer_session') == fields.get('producer_session') and
            bus.get('publication_sequence') == fields.get('publication_sequence')):
        paired = bus.get('failure_origin', '')
    if paired not in ('runtime', 'startup'):
        paired = ''
    if reported and paired and reported != paired:
        return 'unknown', 'conflicting_origins'
    if reported:
        return reported, 'device'
    if paired:
        return paired, 'paired_bus'
    return 'unknown', 'unpaired'


def runtime_fields(fields, level, bus=None):
    """Normalize additive DHI schema 2 without assuming per-ID communication proof."""
    result = dict(fields)
    result.update({
        'schema_version': '1', 'sequence': fields['publication_sequence'],
        'rx_successes': fields['sample_count'], 'dxl_id': fields['id'],
        'rx_failures': fields['read_failures'], 'tx_failures': fields['write_failures'],
        'fault_events': fields['fault_changes'],
        'last_hardware_error': fields['last_hardware_error_status'],
        'diagnostic_level': str(level),
        'rx_sdk_result': fields['read_sdk_result'],
        'tx_sdk_result': fields['write_sdk_result'],
    })
    for key in ('sample_received', 'sample_fresh', 'error_code_available',
                'hardware_error_available', 'estop_available', 'invalid_value'):
        if fields[key] not in ('true', 'false'):
            raise ValueError('invalid boolean field: ' + key)
    collected = (fields['error_code_available'] == 'true' or
                 fields['hardware_error_available'] == 'true')
    result['fault_collected'] = str(collected).lower()
    for direction, legacy in (('rx', 'read'), ('tx', 'write')):
        result[direction + '_result'] = fields[legacy + '_dhi_result']
        raw = fields[legacy + '_sdk_result']
        result[direction + '_reason'] = (
            SDK_RESULTS.get(int(raw), 'SDK_RESULT_UNKNOWN') if raw else 'NO_SDK_OBSERVATION')
    quality = ('MISSING' if fields['sample_received'] != 'true' else
               'FRESH' if fields['sample_fresh'] == 'true' else 'STALE')
    health, reason = 'UNKNOWN', 'NOT_RUNNING'
    if fields['phase'] == 'FAILED':
        origin, source = failure_evidence(fields, bus)
        result.update(failure_origin=origin, failure_origin_source=source)
        quality = 'STALE' if quality != 'MISSING' else quality
        if origin == 'startup':
            health = 'ERROR' if level == 2 else 'UNKNOWN'
            reason = 'INITIALIZATION_FAILED'
        elif origin == 'runtime':
            # A failed hardware component does not identify a broken DXL.
            reason = ('GROUP_READ_FAILED' if int(fields['read_dhi_result']) != 0 else
                      'GROUP_WRITE_FAILED' if int(fields['write_dhi_result']) != 0 else
                      'RUNTIME_HARDWARE_FAILED')
        else:
            reason = 'HARDWARE_FAILURE_ORIGIN_UNKNOWN'
    elif fields['phase'] != 'RUNNING':
        quality = 'STALE' if quality != 'MISSING' else quality
    elif int(fields['read_dhi_result']) != 0:
        reason = 'GROUP_READ_FAILED'
    elif quality == 'MISSING':
        reason = 'NO_CONFIRMED_SAMPLE'
    elif quality == 'STALE':
        reason = 'SAMPLE_EXPIRED'
    elif fields['invalid_value'] == 'true':
        quality, reason = 'INVALID', 'INVALID_REGISTER'
    elif int(fields['error_code']) or int(fields['hardware_error_status']):
        health, reason = 'ERROR', 'DEVICE_REPORTED_FAULT'
    elif int(fields['estop_active']):
        health, reason = 'ERROR', 'ESTOP_ACTIVE'
    elif int(fields['write_dhi_result']) != 0:
        health, reason = 'ERROR', 'GROUP_WRITE_FAILED'
    elif not collected:
        reason = 'FAULT_REGISTER_NOT_CONFIGURED'
    else:
        health, reason = 'OK', 'OBSERVED_OK'
    result.update(health=health, quality=quality, reason=reason)
    for source, available, label, unit in (
        ('error_code', 'error_code_available', 'Error Code', 'raw'),
        ('hardware_error_status', 'hardware_error_available', 'Hardware Error Status', 'bitmask'),
        ('estop_active', 'estop_available', 'E-stop Active', 'bool'),
    ):
        if fields[available] == 'true' and fields['sample_received'] == 'true':
            result['value.' + label] = fields[source]
            result['unit.' + label] = unit
    return result


class DiagnosticStore:
    def __init__(self, expected, robot_id='unassigned', producer_timeout=1.0,
                 history_size=64, startup_run_id=''):
        self.runtime = HealthStore(expected, robot_id, producer_timeout, history_size)
        self.startup_run_id = startup_run_id
        self.startup = {}
        self.buses = {}
        self.rejected = 0

    def ingest_batch(self, records, now):
        """Pair only unique bus/device records in this DiagnosticArray.

        Publication order is irrelevant. Missing, rejected, or duplicate buses
        must not borrow evidence from an earlier array with the same name.
        """
        counts = {}
        for record in records:
            name = record['name']
            counts[name] = counts.get(name, 0) + 1
        valid = [record for record in records if counts[record['name']] == 1]
        self.rejected += len(records) - len(valid)
        # Validate bus session/order/level before making it available to devices.
        buses = {}
        for record in valid:
            if record['name'].endswith('/bus'):
                if self.ingest(**record, now=now):
                    buses[record['name']] = record['fields']
        for record in valid:
            if not record['name'].endswith('/bus'):
                match = DEVICE.fullmatch(record['name'])
                context = buses.get(match[1] + '/bus', {}) if match else {}
                self.ingest(**record, now=now, failure_context=context)

    def ingest(self, name, hardware_id, level, message, fields, now, *, failure_context=None):
        # diagnostic_msgs uses ROS `byte`, which rclpy exposes as one-byte
        # bytes on Jazzy (not JSON-serializable, and not equal to integer 2).
        try:
            level = (level[0] if isinstance(level, (bytes, bytearray)) and len(level) == 1
                     else int(level))
            if level not in (0, 1, 2, 3):
                raise ValueError('invalid diagnostic level')
        except (ValueError, TypeError):
            self.rejected += 1
            return False
        if name.startswith('hat_bridge/'):
            if (name not in {f'hat_bridge/port/{5001+i}' for i in range(5)} or
                    fields.get('hat_bridge_schema') != '1'):
                self.rejected += 1
                return False
            # Additive port components use the existing freshness/history/app
            # contract, while retaining their H5 scope and raw evidence.
            return self.runtime.ingest('dhi/' + name, hardware_id, fields, now)
        if name.startswith('dhi_startup/'):
            if name != 'dhi_startup/observer' and not name.startswith('dhi_startup/invalid/'):
                if (fields.get('startup_schema') != '1' or
                        not fields.get('run_id') or not fields.get('attempt_id') or
                        (self.startup_run_id and fields['run_id'] != self.startup_run_id)):
                    self.rejected += 1
                    return False
            key = (fields.get('run_id', self.startup_run_id), name)
            if key not in self.startup and len(self.startup) >= 128:
                self.rejected += 1
                return False
            self.startup[key] = self._record(name, hardware_id, level, message, fields, now)
            return True
        if fields.get('diagnostics_schema') == '2':
            if name.endswith('/bus'):
                if name not in self.buses and len(self.buses) >= 128:
                    self.rejected += 1
                    return False
                # Use the same session/ordering checks as device records: an older
                # frame must not refresh or replace a newer bus failure.
                old = self.buses.get(name)
                try:
                    session = fields['producer_session']
                    sequence = int(fields['publication_sequence'])
                    if not session or sequence < 1:
                        raise ValueError('invalid bus identity')
                    retired = old.get('retired', []) if old else []
                    if old and (session in retired or (
                            old['evidence']['producer_session'] == session and
                            sequence <= int(old['evidence']['publication_sequence']))):
                        return False
                    if old and old['evidence']['producer_session'] != session:
                        retired = (retired + [old['evidence']['producer_session']])[-8:]
                except (KeyError, ValueError, TypeError):
                    self.rejected += 1
                    return False
                self.buses[name] = self._record(name, hardware_id, level, message, fields, now)
                self.buses[name]['retired'] = retired
                return True
            match = DEVICE.fullmatch(name)
            if match:
                canonical = 'dhi/' + name
                try:
                    if (int(match[2]) != int(fields['comm_id']) or
                            int(match[3]) != int(fields['id'])):
                        raise ValueError('identity mismatch')
                    if failure_context is None:
                        # Compatibility with consumers ingesting the DHI array
                        # sequentially (the reporter emits its bus entry first).
                        bus = self.buses.get(match[1] + '/bus')
                        failure_context = bus['evidence'] if bus else {}
                    normalized = runtime_fields(fields, level, failure_context)
                    normalized['message'] = message
                except (KeyError, ValueError, TypeError):
                    normalized = {'schema_version': 'invalid'}
                return self.runtime.ingest(canonical, hardware_id, normalized, now)
        # Retain support for the other worktree's original schema 1 producer.
        return self.runtime.ingest(name, hardware_id, fields, now)

    @staticmethod
    def _record(name, hardware_id, level, message, fields, now):
        return dict(name=name, hardware_id=hardware_id, level=level, message=message,
                    evidence=dict(fields), received=now)

    def _records(self, entries, now, timeout):
        records = []
        for entry in entries.values():
            record = deepcopy(entry)
            record.pop('retired', None)
            age = max(0.0, now - record.pop('received'))
            record['observer_age_ms'] = age * 1000.0
            record['publisher_alive'] = age <= timeout
            records.append(record)
        return sorted(records, key=lambda r: (r['name'], r['evidence'].get('run_id', '')))

    def snapshot(self, now):
        snapshot, events = self.runtime.snapshot(now)
        snapshot['buses'] = self._records(self.buses, now, self.runtime.timeout)
        snapshot['startup'] = {
            'scope': 'initialization_history_not_runtime_health',
            'run_id_filter': self.startup_run_id,
            # Observer publishes at 1 Hz; do not reuse the faster runtime expiry.
            'records': self._records(self.startup, now, max(3.0, self.runtime.timeout)),
        }
        snapshot['coverage'] = {
            'hardware': 'configured_dhi_observations', 'controllers': 'not_collected',
            'restart_authorized': False,
        }
        snapshot['rejected_messages'] += self.rejected
        return snapshot, events
