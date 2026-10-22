# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Regression tests for the old-name/new-evidence DHI integration boundary."""
from ai_sapiens_diagnostics.adapter import DiagnosticStore
import json
import pytest


NAME = 'k1_left_leg/dxl/200/21'
CANONICAL = 'dhi/' + NAME


def fields(**overrides):
    result = dict(
        diagnostics_schema='2', producer_session='a', publication_sequence='1',
        id='21', comm_id='200', sample_count='1', sample_age_ms='0', stale_after_ms='500',
        sample_received='true', sample_fresh='true', error_code_available='true',
        hardware_error_available='false', estop_available='false', invalid_value='false',
        phase='RUNNING', error_code='0', hardware_error_status='0', estop_active='0',
        read_dhi_result='0', write_dhi_result='0', read_sdk_result='0', write_sdk_result='0',
        read_failures='0', write_failures='0', fault_changes='0', last_error_code='0',
        last_hardware_error_status='0', last_estop_active='0', source_freshness='unknown')
    result.update(overrides)
    return result


def ingest(store, data=None, now=0, level=0):
    return store.ingest(NAME, 'bus:200:21', level, 'test', data or fields(), now)


def component(store, now=0):
    return store.snapshot(now)[0]['components'][0]


def history_fields(**overrides):
    result = dict(fault_occurrences='2', fault_clearances='2',
                  last_fault_started_age_ms='12000', last_fault_cleared_age_ms='5000')
    result.update(overrides)
    return result


def test_fault_history_is_optional_and_not_inferred_from_legacy_counter():
    store = DiagnosticStore([CANONICAL])
    assert ingest(store, fields(fault_changes='12'))
    value = component(store)
    assert value['health'] == 'OK'
    assert value['fault_history']['status'] == 'unsupported'
    assert value['fault_history']['occurrences'] is None


def test_fault_history_ages_locally_and_does_not_keep_current_health_green():
    store = DiagnosticStore([CANONICAL])
    assert ingest(store, fields(**history_fields()), now=10)
    value = component(store, 10.25)
    history = value['fault_history']
    assert value['health'] == 'OK'
    assert history['occurrences'] == history['clearances'] == '2'
    assert history['last_started_age_ms'] == 12250
    assert history['last_cleared_age_ms'] == 5250
    value = component(store, 12)
    assert value['health'] == 'UNKNOWN' and value['reason'] == 'PRODUCER_EXPIRED'
    assert value['fault_history']['clearances'] == '2'
    assert value['fault_history']['last_cleared_age_ms'] == 7000


@pytest.mark.parametrize('overrides', [
    {'fault_occurrences': '-1'}, {'fault_occurrences': '1.5'},
    {'fault_occurrences': 'nan'}, {'fault_occurrences': None},
    {'fault_occurrences': str(2**64)}, {'fault_occurrences': '２'},
    {'fault_occurrences': '0'}, {'fault_clearances': '3'},
    {'last_fault_started_age_ms': '-2'}, {'last_fault_cleared_age_ms': '-1'},
    {'last_fault_cleared_age_ms': 'inf'}, {'last_fault_cleared_age_ms': str(2**63)},
])
def test_bad_optional_history_does_not_hide_valid_current_device_fault(overrides):
    store = DiagnosticStore([CANONICAL])
    assert ingest(store, fields(error_code='4', **history_fields(**overrides)), level=2)
    value = component(store)
    assert value['health'] == 'ERROR' and value['reason'] == 'DEVICE_REPORTED_FAULT'
    assert value['fault_history']['status'] == 'invalid'
    assert value['fault_history']['occurrences'] is None
    json.dumps(value, allow_nan=False)


def test_partial_history_is_invalid_but_zero_history_and_uint64_counts_are_supported():
    store = DiagnosticStore([CANONICAL])
    assert ingest(store, fields(fault_occurrences='1'))
    assert component(store)['fault_history']['status'] == 'invalid'
    assert ingest(store, fields(publication_sequence='2', **history_fields(
        fault_occurrences='0', fault_clearances='0',
        last_fault_started_age_ms='-1', last_fault_cleared_age_ms='-1')))
    history = component(store)['fault_history']
    assert history['status'] == 'available' and history['occurrences'] == '0'
    assert history['last_started_age_ms'] is None and history['last_cleared_age_ms'] is None
    count = str(2**64 - 1)
    assert ingest(store, fields(publication_sequence='3', **history_fields(
        fault_occurrences=count, fault_clearances=count)))
    assert component(store)['fault_history']['occurrences'] == count


def test_clearance_creates_event_even_if_current_health_and_old_fault_counter_are_unchanged():
    store = DiagnosticStore([CANONICAL])
    assert ingest(store, fields(**history_fields(fault_clearances='1')))
    store.snapshot(0)
    assert ingest(store, fields(publication_sequence='2', sample_count='2',
                               **history_fields()), now=0.1)
    _, events = store.snapshot(0.1)
    assert len(events) == 1 and events[0]['health'] == 'OK'
    assert events[0]['fault_history']['clearances'] == '2'
    assert store.snapshot(0.2)[1] == []  # Age alone must not flood event history.
    assert ingest(store, fields(publication_sequence='3', sample_count='3',
                               **history_fields(last_fault_cleared_age_ms='5200')), now=0.3)
    assert store.snapshot(0.3)[1] == []


def test_new_session_resets_fault_history_and_retired_packets_cannot_restore_old_counts():
    store = DiagnosticStore([CANONICAL])
    assert ingest(store, fields(**history_fields()))
    assert ingest(store, fields(producer_session='b', **history_fields(
        fault_occurrences='0', fault_clearances='0',
        last_fault_started_age_ms='-1', last_fault_cleared_age_ms='-1')), now=1)
    assert not ingest(store, fields(publication_sequence='2', **history_fields()), now=1.1)
    assert component(store, 1.1)['fault_history']['occurrences'] == '0'
    assert component(store, 1.1)['producer_session'] == 'b'


def test_original_name_maps_by_comm_and_device_not_gpio_alias():
    store = DiagnosticStore([CANONICAL])
    assert ingest(store)
    value = component(store)
    assert value['name'] == CANONICAL and value['health'] == 'OK'
    assert value['dxl_id'] == '21' and value['evidence']['comm_id'] == '200'


def test_heartbeat_without_rx_progress_expires_sample():
    store = DiagnosticStore([CANONICAL])
    ingest(store)
    ingest(store, fields(publication_sequence='2'), 0.6)
    assert component(store, 0.6)['reason'] == 'SAMPLE_EXPIRED'


def test_priming_missing_data_and_inactive_never_claim_healthy():
    for overrides in (
        dict(sample_count='0', sample_received='false', sample_fresh='false', sample_age_ms='-1'),
        dict(phase='INACTIVE'), dict(phase='READY_NO_RUNTIME_SAMPLE'),
    ):
        store = DiagnosticStore([CANONICAL])
        assert ingest(store, fields(**overrides))
        assert component(store)['health'] == 'UNKNOWN'


def test_group_timeout_preserves_id_fault_without_claiming_id_is_offline():
    store = DiagnosticStore([CANONICAL])
    ingest(store, fields(error_code='4', fault_changes='1', last_error_code='4'))
    ingest(store, fields(publication_sequence='2', sample_fresh='false',
                         error_code='4', read_dhi_result='-7', read_sdk_result='-3001',
                         read_failures='1', fault_changes='1', last_error_code='4'), 0.1)
    value = component(store, 0.1)
    assert value['reason'] == 'GROUP_READ_FAILED'
    assert value['health'] == 'UNKNOWN'
    assert value['evidence']['rx_reason'] == 'RX_TIMEOUT'
    assert value['evidence']['last_error_code'] == '4'


def test_write_failure_is_group_evidence_not_command_acknowledgement():
    store = DiagnosticStore([CANONICAL])
    ingest(store, fields(write_dhi_result='-6', write_sdk_result='-1001', write_failures='1',
                         command_applied='unknown'))
    value = component(store)
    assert value['health'] == 'ERROR' and value['reason'] == 'GROUP_WRITE_FAILED'
    assert value['evidence']['command_applied'] == 'unknown'


def test_estop_and_recovery_retain_fault_evidence():
    store = DiagnosticStore([CANONICAL])
    ingest(store, fields(estop_available='true', estop_active='1', last_estop_active='1',
                         fault_changes='1'))
    assert component(store)['reason'] == 'ESTOP_ACTIVE'
    ingest(store, fields(publication_sequence='2', sample_count='2', estop_available='true',
                         last_estop_active='1', fault_changes='1'), 0.1)
    value = component(store, 0.1)
    assert value['health'] == 'OK' and value['evidence']['last_estop_active'] == '1'
    assert store.snapshot(0.1)[0]['coverage']['restart_authorized'] is False


def test_imu_without_fault_register_remains_unknown_but_rx_is_visible():
    store = DiagnosticStore([CANONICAL])
    ingest(store, fields(error_code_available='false'))
    value = component(store)
    assert value['health'] == 'UNKNOWN'
    assert value['quality'] == 'FRESH'
    assert value['reason'] == 'FAULT_REGISTER_NOT_CONFIGURED'


def test_malformed_identity_invalidates_previous_runtime_entry():
    store = DiagnosticStore([CANONICAL])
    ingest(store)
    assert not ingest(store, fields(id='22', publication_sequence='2'), 0.1)
    assert component(store, 0.1)['reason'] == 'INVALID_DIAGNOSTIC'


def test_startup_success_is_history_not_live_health_and_filters_other_runs():
    store = DiagnosticStore([CANONICAL], startup_run_id='run-a')
    data = dict(startup_schema='1', run_id='run-a', attempt_id='a',
                phase='READY_NO_RUNTIME_SAMPLE')
    assert store.ingest('dhi_startup/k1_left_leg', 'bus', 0, 'init complete', data, 0)
    state, _ = store.snapshot(4)
    assert state['health'] == 'UNKNOWN'
    assert state['startup']['records'][0]['publisher_alive'] is False
    assert state['startup']['records'][0]['evidence']['phase'] == 'READY_NO_RUNTIME_SAMPLE'
    assert not store.ingest('dhi_startup/k1_left_leg', 'bus', 2, 'old failure',
                            dict(data, run_id='run-b'), 5)


def test_startup_failure_retained_after_observer_dies():
    store = DiagnosticStore([CANONICAL])
    data = dict(startup_schema='1', run_id='run', attempt_id='a', phase='FAILED',
                stage='device_init', target_id='21', target_comm_id='200')
    store.ingest('dhi_startup/k1_left_leg', 'bus', 2, 'init failed', data, 0)
    record = store.snapshot(100)[0]['startup']['records'][0]
    assert record['level'] == 2 and record['evidence']['target_id'] == '21'
    assert not record['publisher_alive']


def test_ros_byte_level_is_normalized_before_json_and_failure_assessment():
    store = DiagnosticStore([CANONICAL])
    assert ingest(store, fields(phase='FAILED', failure_origin='startup'), level=b'\x02')
    assert component(store)['health'] == 'ERROR'
    data = dict(startup_schema='1', run_id='run', attempt_id='a', phase='FAILED')
    assert store.ingest('dhi_startup/k1_left_leg', 'bus', b'\x02', 'failed', data, 0)
    state, _ = store.snapshot(0)
    assert state['startup']['records'][0]['level'] == 2
    json.dumps(state, allow_nan=False)


def test_bus_replay_does_not_refresh_newer_failure_and_restart_accepts_counter_reset():
    store = DiagnosticStore([CANONICAL])

    def bus(seq, session='a'):
        return fields(publication_sequence=str(seq), producer_session=session)
    store.ingest('k1_left_leg/bus', 'bus', 2, 'read failed', bus(4), 0)
    assert not store.ingest('k1_left_leg/bus', 'bus', 0, 'ok', bus(3), 0.9)
    assert not store.snapshot(1.1)[0]['buses'][0]['publisher_alive']
    assert store.ingest('k1_left_leg/bus', 'bus', 0, 'new', bus(1, 'b'), 2)
    assert not store.ingest('k1_left_leg/bus', 'bus', 2, 'old', bus(5), 2.1)


def failure_records(origin='runtime', **overrides):
    data = fields(phase='FAILED', sample_fresh='false', read_dhi_result='-7',
                  read_sdk_result='-3002')
    data.update(overrides)
    bus = dict(name='k1_left_leg/bus', hardware_id='port', level=2,
               message='Hardware failed', fields=dict(data, failure_origin=origin))
    device = dict(name=NAME, hardware_id='port:200:21', level=3,
                  message='Group read failed; state unknown', fields=data)
    return bus, device


@pytest.mark.parametrize('reverse', [False, True])
def test_runtime_failed_component_is_not_an_initialization_or_individual_motor_fault(reverse):
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records()
    store.ingest_batch([device, bus] if reverse else [bus, device], now=0)
    value = component(store)
    assert value['reason'] == 'GROUP_READ_FAILED'
    assert value['health'] == 'UNKNOWN' and value['quality'] == 'STALE'
    assert value['evidence']['failure_origin'] == 'runtime'
    assert value['evidence']['failure_origin_source'] == 'paired_bus'
    assert value['evidence']['diagnostic_level'] == '3'
    assert value['evidence']['rx_reason'] == 'RX_CORRUPT'
    assert store.snapshot(0)[0]['coverage']['restart_authorized'] is False


def test_real_startup_target_error_and_unchecked_devices_are_preserved():
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records('startup')
    device['level'] = 2
    store.ingest_batch([bus, device], now=0)
    assert component(store)['reason'] == 'INITIALIZATION_FAILED'
    assert component(store)['health'] == 'ERROR'
    bus, device = failure_records('startup', publication_sequence='2')
    store.ingest_batch([bus, device], now=0.1)
    assert component(store, 0.1)['health'] == 'UNKNOWN'
    assert component(store, 0.1)['reason'] == 'INITIALIZATION_FAILED'


@pytest.mark.parametrize('change', [
    {'publication_sequence': '2'}, {'producer_session': 'other'},
    {'phase': 'RUNNING'}, {'failure_origin': ''}, {'failure_origin': 'guess'},
])
def test_unpaired_bus_cannot_claim_failure_origin(change):
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records()
    bus['fields'].update(change)
    store.ingest_batch([bus, device], now=0)
    value = component(store)
    assert value['reason'] == 'HARDWARE_FAILURE_ORIGIN_UNKNOWN'
    assert value['health'] == 'UNKNOWN'
    assert value['evidence']['failure_origin_source'] == 'unpaired'


def test_missing_current_bus_does_not_borrow_prior_array_evidence():
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records()
    store.ingest_batch([bus], now=0)
    store.ingest_batch([device], now=0.1)
    assert component(store, 0.1)['reason'] == 'HARDWARE_FAILURE_ORIGIN_UNKNOWN'


@pytest.mark.parametrize('bad_bus', ['duplicate', 'invalid_level', 'invalid_identity', 'duplicate_device'])
def test_rejected_records_cannot_supply_correlated_failure_evidence(bad_bus):
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records()
    records = [bus, device]
    if bad_bus == 'duplicate':
        records.insert(0, dict(bus, fields=dict(bus['fields'], failure_origin='startup')))
    elif bad_bus == 'duplicate_device':
        records.append(dict(device))
    elif bad_bus == 'invalid_level':
        bus['level'] = 255
    else:
        bus['fields']['publication_sequence'] = '0'
    store.ingest_batch(records, now=0)
    value = component(store)
    assert value['health'] == 'UNKNOWN'
    if bad_bus == 'duplicate_device':
        assert value['reason'] == 'PRODUCER_MISSING'
    else:
        assert value['reason'] == 'HARDWARE_FAILURE_ORIGIN_UNKNOWN'
    assert store.rejected >= 1


@pytest.mark.parametrize('read,write,reason', [
    ('0', '-6', 'GROUP_WRITE_FAILED'), ('0', '0', 'RUNTIME_HARDWARE_FAILED'),
])
def test_runtime_stop_without_current_read_failure_remains_group_evidence(read, write, reason):
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records(read_dhi_result=read, write_dhi_result=write)
    store.ingest_batch([bus, device], now=0)
    assert component(store)['reason'] == reason
    assert component(store)['health'] == 'UNKNOWN'


@pytest.mark.parametrize('reported,source', [
    ('startup', 'conflicting_origins'), ('invalid', 'invalid_device_origin'),
])
def test_conflicting_or_invalid_origin_cannot_invent_initialization_failure(reported, source):
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records()
    device['fields']['failure_origin'] = reported
    # Give the bus independent fields, as real separate DiagnosticStatus records have.
    store.ingest_batch([bus, device], now=0)
    assert component(store)['reason'] == 'HARDWARE_FAILURE_ORIGIN_UNKNOWN'
    assert component(store)['evidence']['failure_origin_source'] == source


def test_sequential_ingest_uses_only_exact_publication_and_keeps_runtime_evidence():
    store = DiagnosticStore([CANONICAL])
    bus, device = failure_records()
    store.ingest(**bus, now=0)
    store.ingest(**device, now=0)
    assert component(store)['reason'] == 'GROUP_READ_FAILED'
    device['fields']['publication_sequence'] = '2'
    store.ingest(**device, now=0.1)
    assert component(store, 0.1)['reason'] == 'HARDWARE_FAILURE_ORIGIN_UNKNOWN'
