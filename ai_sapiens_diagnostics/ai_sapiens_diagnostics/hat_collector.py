"""Non-ROS policy for bounded HAT collection and app-compatible evidence."""
from collections import OrderedDict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
import uuid

from .hat_protocol import H5Client, collect

DEFAULT_BUSES = ('k1_right_arm/bus', 'k1_left_arm/bus', 'k1_left_leg/bus',
                 'k1_right_leg/bus', 'k1_imu/bus')


class FailureTrigger:
    """At most one pending onset per configured bus, bounded retired sessions.

    This consumes the non-real-time DiagnosticArray reporter, not DHI read/write.
    Repeated failed publications never issue one H5 query per failed cycle.
    """
    def __init__(self, buses=DEFAULT_BUSES):
        if len(buses) != 5 or len(set(buses)) != 5:
            raise ValueError('five unique H5 channel-to-bus mappings required')
        self.channels = {name: i for i, name in enumerate(buses)}
        self.state = {}
        self.pending = OrderedDict()

    def ingest(self, name, fields, now=None):
        if name not in self.channels or fields.get('diagnostics_schema') != '2':
            return False
        try:
            session = fields['producer_session']
            sequence = int(fields['publication_sequence'])
            failures = int(fields['read_failures'])
            read_result = int(fields['read_dhi_result'])
            if not session or sequence < 1 or failures < 0:
                return False
        except (KeyError, ValueError, TypeError):
            return False
        old = self.state.get(name)
        retired = old['retired'] if old else []
        if session in retired or (old and session == old['session'] and sequence <= old['sequence']):
            return False
        same = old and session == old['session']
        # Device-init/PING failure is a host failure, even before read begins.
        startup = fields.get('phase') == 'FAILED' and fields.get('failure_origin') == 'startup'
        failed = read_result != 0 or startup
        increased = bool(same and failures > old['failures'])
        onset = (failed and not (same and old['failed'])) or (increased and not (same and old['failed']))
        if old and not same:
            retired = (retired + [old['session']])[-8:]
        self.state[name] = dict(session=session, sequence=sequence, failures=failures,
                                failed=failed, retired=retired)
        if onset and name not in self.pending:
            self.pending[name] = dict(name=name, channel=self.channels[name],
                host_observed_utc=datetime.now(timezone.utc).isoformat(),
                host_observed_monotonic_ns=time.monotonic_ns() if now is None else now,
                producer_session=session, publication_sequence=sequence,
                read_failures=failures, read_dhi_result=read_result,
                read_sdk_result=fields.get('read_sdk_result'), phase=fields.get('phase'),
                response_evidence_sequence=fields.get('response_evidence_sequence'))
        return onset

    def take(self):
        pending = list(self.pending.values())
        self.pending.clear()
        return sum(1 << item['channel'] for item in pending), pending


class EvidenceFiles:
    """Atomic, bounded files owned by this collector; never touch legacy logs."""
    def __init__(self, directory, capacity=128):
        if capacity < 2 or capacity > 4096:
            raise ValueError('evidence capacity must be 2..4096')
        self.directory = Path(directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.capacity = capacity
        self.evicted = 0

    def update_latest(self, evidence):
        """Atomic current query status; immutable history uses save()."""
        raw = json.dumps(evidence, ensure_ascii=False, sort_keys=True)
        if len(raw.encode()) > 262144:
            raise ValueError('latest evidence exceeds bounded file size')
        temporary = self.directory / 'latest.tmp'
        final = self.directory / 'latest.json'
        try:
            with temporary.open('w', encoding='utf-8') as stream:
                stream.write(raw + '\n')
            temporary.replace(final)
        finally:
            temporary.unlink(missing_ok=True)
        return str(final)

    def save(self, evidence):
        evidence['collector_retention'] = dict(capacity=self.capacity, evicted=self.evicted)
        raw = json.dumps(evidence, ensure_ascii=False, sort_keys=True, indent=2)
        if len(raw.encode()) > 262144:
            raise ValueError('evidence exceeds bounded file size')
        stem = 'hat-evidence-' + str(time.time_ns()) + '-' + uuid.uuid4().hex[:8]
        temporary = self.directory / (stem + '.tmp')
        final = self.directory / (stem + '.json')
        try:
            with temporary.open('x', encoding='utf-8') as stream:
                stream.write(raw + '\n')
                stream.flush()
                os.fsync(stream.fileno())
            temporary.replace(final)
        finally:
            temporary.unlink(missing_ok=True)
        files = sorted(self.directory.glob('hat-evidence-*.json'))
        for stale in files[:-self.capacity]:
            stale.unlink()
            self.evicted += 1
        return str(final)


def stage_observations(live, previous):
    if not previous or live['boot_id'] != previous['boot_id']:
        return ['NO_PREVIOUS_CAPTURE_FOR_COMPARISON']
    stages, old = live['stages'], previous['stages']
    # These are interval observations, not per-request response correlation.
    if stages['request']['count'] == old['request']['count']:
        return ['UDP_REQUEST_NOT_OBSERVED_IN_CAPTURE_INTERVAL']
    observations = []
    if stages['uart_tc']['count'] == old['uart_tc']['count']:
        observations.append('UART_TX_COMPLETION_NOT_OBSERVED_IN_CAPTURE_INTERVAL')
    if stages['rx']['bytes'] == old['rx']['bytes']:
        observations.append('UART_RX_NOT_OBSERVED_IN_CAPTURE_INTERVAL')
    if stages['net_done']['count'] == old['net_done']['count']:
        observations.append('UDP_RETURN_COMPLETION_NOT_OBSERVED_IN_CAPTURE_INTERVAL')
    return observations or ['BIDIRECTIONAL_PROGRESS_OBSERVED_IN_CAPTURE_INTERVAL']


def diagnostic_records(evidence, previous, session, sequence, successes, file_path=''):
    """Feed existing HealthStore/app evidence without pretending to be a DXL."""
    records = []
    for i in range(5):
        good = evidence.get('complete', False)
        live = evidence['live'][i] if good else None
        old = previous['live'][i] if previous and previous.get('complete') else None
        first = evidence['first'][i] if good else None
        observations = stage_observations(live, old) if good else ['H5_QUERY_UNAVAILABLE']
        reason = 'H5_EVIDENCE_AVAILABLE' if good else 'H5_QUERY_FAILED_OR_UNSUPPORTED'
        health, level = 'UNKNOWN', 3
        if good:
            if ((evidence.get('failure_mask', 0) & (1 << i)) or
                    (live['occurrences'] and (not old or live['boot_id'] != old['boot_id'] or
                                             live['occurrences'] != old['occurrences']))):
                health, level, reason = 'WARN', 1, 'H5_FAILURE_EVIDENCE_CAPTURED'
            elif observations == ['BIDIRECTIONAL_PROGRESS_OBSERVED_IN_CAPTURE_INTERVAL']:
                health, level, reason = 'OK', 0, 'H5_BRIDGE_PROGRESS_OBSERVED'
        fields = dict(schema_version='1', hat_bridge_schema='1', producer_session=session, sequence=str(sequence),
            rx_successes=str(successes), sample_age_ms='0' if successes else '-1',
            stale_after_ms='30000', health=health, quality='FRESH' if good else 'MISSING',
            fault_collected=str(good).lower(), component_type='hat_bridge_port',
            reason=reason, message=reason, bus=str(5001+i), source_freshness='collector_receipt',
            evidence_file=file_path, host_observed_utc=evidence.get('host_started_utc', ''),
            scope='H5 bridge observations; motor and physical wire health not established',
            stage_observations=json.dumps(observations),
            query_errors=json.dumps(evidence.get('errors', [])),
            fault_events=str(live['occurrences'] if good else 0),
            capture_drops=str(evidence.get('file_errors', 0)))
        if good:
            fields.update(boot_id=str(live['boot_id']), mcu_uptime_ms=str(live['uptime_ms']),
                          first_event_sequence=str(first['event_sequence']),
                          first_trigger=first['trigger_name'],
                          stage_counters=json.dumps(live['stages'], sort_keys=True),
                          applied_baud=str(live['applied_baud']), tx_state=str(live['tx_state']),
                          uart_isr=str(live['uart_isr']), fault_reason_mask=str(live['reason_mask']),
                          journal_status=json.dumps(evidence['journal_status'], sort_keys=True))
        records.append(dict(name=f'hat_bridge/port/{5001+i}',
                            hardware_id=f'H5:{evidence.get("service_id")}:channel:{i}',
                            level=level, message=reason, fields=fields))
    return records


def main():
    """Standalone one-shot export; ROS-independent for development/inspection."""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--id', type=int, required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--read-only', action='store_true', help='disable RAM freeze and host trigger')
    args = parser.parse_args()
    with H5Client(args.host, args.id) as client:
        result = collect(client, allow_freeze=not args.read_only)
    print(EvidenceFiles(args.output).save(result))
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
