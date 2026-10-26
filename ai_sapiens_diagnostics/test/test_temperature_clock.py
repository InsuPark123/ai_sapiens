"""Clock rollback must allow new frames without admitting out-of-order frames."""
from types import SimpleNamespace

import pytest

from ai_sapiens_diagnostics.motor_temperature import MotorTemperatures, ImuTemperature
from ai_sapiens_diagnostics.temperature_node import TemperatureNode


@pytest.fixture(params=[MotorTemperatures, ImuTemperature])
def sample(request):
    cache = request.param('k1-test')
    identity = next(iter(cache.devices))
    name = 'imu' if isinstance(cache, ImuTemperature) else f'dxl{identity}'
    source_name = f'{cache.devices[identity]}/dxl/{identity}/{identity}'
    def source(sequence, session='session'):
        cache.ingest_source(source_name, dict(
            producer_session=session, publication_sequence=str(sequence),
            sample_count=str(sequence), sample_age_ms='0', diagnostics_schema='2',
            sample_fresh='true', sample_received='true', read_dhi_result='0'))
    def message(seconds):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=0)),
            joint_names=[name], interface_values=[SimpleNamespace(
                interface_names=cache.interfaces, values=[25.0] * len(cache.interfaces))])
    clock = SimpleNamespace(nanoseconds=100_000_000_000)
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: clock))
    source(1)
    source(2)
    TemperatureNode.ingest_dynamic(node, cache, message(100))
    assert cache.snapshot()[0]['level'] == 0
    return cache, clock, node, message, source


def test_clock_rollback_accepts_new_epoch_and_rejects_future_old_epoch(sample):
    cache, clock, node, message, _ = sample
    clock.nanoseconds = 1_000_000_000
    TemperatureNode.ingest_dynamic(node, cache, message(100))
    assert cache.snapshot()[0]['level'] == 3
    TemperatureNode.ingest_dynamic(node, cache, message(1))
    assert cache.snapshot()[0]['level'] == 0
    assert cache.dynamic_stamp == 1_000_000_000


def test_reordered_and_duplicate_frames_do_not_reset_epoch(sample):
    cache, clock, node, message, _ = sample
    clock.nanoseconds = 100_100_000_000
    cached = dict(cache.samples)
    TemperatureNode.ingest_dynamic(node, cache, message(100))
    assert cache.samples == cached
    # A valid-age but older frame must not reset the timestamp guard.
    older = message(99)
    older.header.stamp.nanosec = 900_000_000
    TemperatureNode.ingest_dynamic(node, cache, older)
    assert cache.samples == cached
    assert cache.dynamic_stamp == 100_000_000_000


def test_rollback_does_not_bypass_source_restart_freshness(sample):
    cache, clock, node, message, source = sample
    source(1, 'reboot')
    clock.nanoseconds = 1_000_000_000
    TemperatureNode.ingest_dynamic(node, cache, message(1))
    assert cache.snapshot()[0]['level'] == 3
    source(2, 'reboot')
    clock.nanoseconds = 1_200_000_000
    new = message(1)
    new.header.stamp.nanosec = 200_000_000
    TemperatureNode.ingest_dynamic(node, cache, new)
    assert cache.snapshot()[0]['level'] == 0
