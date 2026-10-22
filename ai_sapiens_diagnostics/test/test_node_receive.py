# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Actual DiagnosticArray receive contracts; no hardware or ROS graph required."""
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ai_sapiens_diagnostics.adapter import DiagnosticStore
from ai_sapiens_diagnostics.node import HealthNode
from test_adapter import CANONICAL, NAME, fields


def status(name, data, level=0):
    return DiagnosticStatus(name=name, hardware_id='synthetic', level=bytes([level]),
                            values=[KeyValue(key=k, value=v) for k, v in data.items()])


def receive(monkeypatch, store, rows):
    monkeypatch.setattr('ai_sapiens_diagnostics.node.time.monotonic', lambda: 0.0)
    HealthNode.receive(SimpleNamespace(store=store), DiagnosticArray(status=rows))
    return store.snapshot(0.0)[0]


@pytest.mark.parametrize('overrides,level,health,quality,reason', [
    ({}, 0, 'OK', 'FRESH', 'OBSERVED_OK'),
    ({'error_code': '4'}, 2, 'ERROR', 'FRESH', 'DEVICE_REPORTED_FAULT'),
    ({'read_dhi_result': '-7', 'sample_fresh': 'false'}, 3,
     'UNKNOWN', 'STALE', 'GROUP_READ_FAILED'),
    ({'write_dhi_result': '-6'}, 2, 'ERROR', 'FRESH', 'GROUP_WRITE_FAILED'),
])
def test_running_array_preserves_fault_and_communication_meanings(
        monkeypatch, overrides, level, health, quality, reason):
    data = fields(**overrides)
    store = DiagnosticStore([CANONICAL])
    snapshot = receive(monkeypatch, store, [
        status(NAME, data, level), status('k1_left_leg/bus', data, level)])
    value = snapshot['components'][0]
    assert (value['health'], value['quality'], value['reason']) == (health, quality, reason)
    assert value['evidence']['diagnostic_level'] == str(level)
    assert snapshot['coverage']['restart_authorized'] is False


def test_normal_failure_and_recovery_use_only_same_array_origin(monkeypatch):
    store = DiagnosticStore([CANONICAL])
    failure = fields(phase='FAILED', sample_fresh='false', read_dhi_result='-7')
    # Device before its bus must still use this array's validated origin.
    snapshot = receive(monkeypatch, store, [status(NAME, failure, 3),
        status('k1_left_leg/bus', dict(failure, failure_origin='runtime'), 2)])
    assert snapshot['components'][0]['reason'] == 'GROUP_READ_FAILED'
    normal = fields(publication_sequence='2', sample_count='2')
    snapshot = receive(monkeypatch, store, [status(NAME, normal),
                                          status('k1_left_leg/bus', normal)])
    assert snapshot['components'][0]['reason'] == 'OBSERVED_OK'
    failure['publication_sequence'] = '3'
    failure['sample_count'] = '3'
    snapshot = receive(monkeypatch, store, [status(NAME, failure, 3)])
    value = snapshot['components'][0]
    assert value['reason'] == 'HARDWARE_FAILURE_ORIGIN_UNKNOWN'
    assert value['evidence']['failure_origin_source'] == 'unpaired'


@pytest.mark.parametrize('ambiguity', ['name', 'key'])
def test_ambiguous_normal_device_never_creates_a_healthy_observation(monkeypatch, ambiguity):
    store = DiagnosticStore([CANONICAL])
    device = status(NAME, fields())
    rows = [device]
    if ambiguity == 'name':
        rows.append(status(NAME, fields(error_code='4'), 2))
    else:
        device.values.append(KeyValue(key='error_code', value='4'))
    snapshot = receive(monkeypatch, store, rows)
    assert snapshot['components'][0]['health'] == 'UNKNOWN'
    assert snapshot['components'][0]['quality'] == 'MISSING'
    assert store.rejected == (2 if ambiguity == 'name' else 1)
