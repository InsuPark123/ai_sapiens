# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
import json

from ai_sapiens_diagnostics.health import HealthStore
import pytest

NAME = 'dhi/k1_mock/dxl21'
OTHER = 'dhi/k1_mock/dxl22'


def fields(sequence=1, **overrides):
    value = {
        'schema_version': '1', 'producer_session': 'a', 'sequence': str(sequence),
        'rx_successes': str(sequence), 'sample_age_ms': '0', 'stale_after_ms': '250',
        'health': 'OK', 'quality': 'FRESH', 'reason': 'OBSERVED_OK', 'fault_collected': 'true',
        'fault_events': '0', 'rx_failures': '0', 'tx_failures': '0', 'capture_drops': '0',
        'source_freshness': 'unknown',
    }
    value.update(overrides)
    return value


def component(store, now=0):
    return store.snapshot(now)[0]['components'][0]


def test_missing_expected_devices_are_unknown():
    state, events = HealthStore([NAME]).snapshot(0)
    assert state['health'] == 'UNKNOWN'
    assert state['unknown_required'] == [NAME]
    assert events[0]['reason'] == 'PRODUCER_MISSING'


def test_healthy_requires_all_required_components():
    store = HealthStore([NAME, OTHER])
    store.ingest(NAME, 'mock', fields(), 0)
    assert store.snapshot(0)[0]['health'] == 'UNKNOWN'
    store.ingest(OTHER, 'mock', fields(), 0)
    assert store.snapshot(0)[0]['health'] == 'OK'


def test_dxl_fault_does_not_change_other_device():
    store = HealthStore([NAME, OTHER])
    store.ingest(NAME, 'mock', fields(health='ERROR', reason='DEVICE_REPORTED_FAULT'), 0)
    store.ingest(OTHER, 'mock', fields(), 0)
    state, _ = store.snapshot(0)
    assert state['known_faults'] == [NAME]
    assert state['components'][1]['health'] == 'OK'


def test_duplicate_and_out_of_order_do_not_keep_producer_alive():
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(2), 0)
    assert not store.ingest(NAME, 'mock', fields(2), 0.9)
    assert not store.ingest(NAME, 'mock', fields(1), 1.1)
    assert component(store, 1.2)['reason'] == 'PRODUCER_EXPIRED'


def test_publishing_without_rx_progress_cannot_refresh_sample():
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(1), 0)
    store.ingest(NAME, 'mock', fields(2, rx_successes='1'), 0.3)
    assert component(store, 0.3)['reason'] == 'SAMPLE_EXPIRED'


def test_new_responses_with_unchanged_values_are_valid():
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(1), 0)
    store.ingest(NAME, 'mock', fields(2), 0.3)
    assert component(store, 0.3)['health'] == 'OK'


def test_source_age_is_preserved_and_extended_locally():
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(sample_age_ms='200'), 0)
    assert component(store, 0.1)['reason'] == 'SAMPLE_EXPIRED'


def test_session_restart_accepts_reset_sequence_and_rejects_retired_session():
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(500), 0)
    store.ingest(NAME, 'mock', fields(1, producer_session='b'), 1)
    assert component(store, 1)['health'] == 'OK'
    assert not store.ingest(NAME, 'mock', fields(501), 1.1)
    assert component(store, 1.1)['producer_session'] == 'b'


@pytest.mark.parametrize('overrides', [
    {'sample_age_ms': 'nan'}, {'stale_after_ms': '0'}, {'schema_version': '2'},
    {'health': 'unrecognized'}, {'sequence': '-1'}, {'quality': 'MISSING'},
    {'fault_collected': 'false'}, {'rx_successes': '-1'}, {'fault_events': '-1'},
])
def test_malformed_payload_invalidates_previous_state(overrides):
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(), 0)
    data = fields(2)
    data.update(overrides)
    assert not store.ingest(NAME, 'mock', data, 0.1)
    assert component(store, 0.1)['reason'] == 'INVALID_DIAGNOSTIC'


def test_failure_counters_preserve_transient_event_after_recovery():
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(), 0)
    store.snapshot(0)
    store.ingest(NAME, 'mock', fields(2, fault_events='1', last_error_code='3'), 0.1)
    state, events = store.snapshot(0.1)
    assert state['health'] == 'OK'
    assert events[0]['evidence']['last_error_code'] == '3'


def test_event_history_is_bounded_and_reports_loss():
    store = HealthStore([NAME], history_size=2)
    for i in range(1, 5):
        store.ingest(NAME, 'mock', fields(i, fault_events=str(i)), i)
        state, _ = store.snapshot(i)
    assert len(state['recent_events']) == 2
    assert state['history_dropped_total'] == 2


def test_unknown_and_fault_can_coexist_in_summary():
    store = HealthStore([NAME, OTHER])
    store.ingest(NAME, 'mock', fields(health='ERROR'), 0)
    state, _ = store.snapshot(0)
    assert state['health'] == 'ERROR'
    assert state['unknown_required'] == [OTHER]


def test_nonfinite_values_are_json_null_not_nan():
    store = HealthStore([NAME])
    data = fields(health='UNKNOWN', quality='INVALID')
    data['value.Present Current'] = 'nan'
    data['unit.Present Current'] = 'N m'
    store.ingest(NAME, 'mock', data, 0)
    state, _ = store.snapshot(0)
    value = state['components'][0]['measurements']['Present Current']
    assert value == {'value': None, 'unit': 'N m', 'quality': 'INVALID'}
    json.dumps(state, allow_nan=False)


def test_zero_sample_missing_state_and_no_inventory_are_not_healthy():
    store = HealthStore([NAME])
    store.ingest(NAME, 'mock', fields(
        rx_successes='0', sample_age_ms='-1', health='UNKNOWN', quality='MISSING'), 0)
    assert component(store)['sample_age_ms'] is None
    assert HealthStore([]).snapshot(0)[0]['health'] == 'UNKNOWN'
