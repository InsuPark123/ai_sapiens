"""IMU transport freshness, signed Celsius and separation from motor data."""
import pytest
from ai_sapiens_diagnostics.motor_temperature import ImuTemperature, MotorTemperatures


def source(sequence=1, count=1, **extra):
    fields = dict(producer_session='imu-session', publication_sequence=str(sequence),
                  sample_count=str(count), sample_age_ms='0', diagnostics_schema='2',
                  sample_fresh='true', sample_received='true', read_dhi_result='0')
    fields.update(extra)
    return fields


def prime(cache, value=25.25):
    cache.ingest_source('k1_imu/dxl/202/202', source(), now=10)
    cache.ingest_source('k1_imu/dxl/202/202', source(2, 201), now=10.2)
    cache.ingest_dynamic([('imu', ['Temperature'], [value])], stamp=1, now=10.3)


@pytest.mark.parametrize('value', [-327.68, -12.34, 0.0, 25.25, 327.67])
def test_signed_celsius_is_not_scaled_again(value):
    cache = ImuTemperature('k1-test')
    prime(cache, value)
    row, = cache.snapshot(10.4)
    assert row['name'] == 'temperature/k1_imu/dxl/202'
    assert row['level'] == 0
    assert row['fields']['imu_temperature_c'] == str(value)
    assert row['fields']['unit'] == 'degC'
    assert row['fields']['firmware_temperature_validity_evaluated'] == 'false'
    assert 'motor_temperature_c' not in row['fields']


def test_cached_temperature_cannot_keep_stopped_source_fresh():
    cache = ImuTemperature('k1-test')
    prime(cache)
    cache.ingest_dynamic([('imu', ['Temperature'], [25.25])], stamp=2, now=11)
    assert cache.snapshot(11)[0]['level'] == 3


def test_session_restart_and_read_error_require_new_temperature_frame():
    cache = ImuTemperature('k1-test')
    prime(cache)
    cache.ingest_source('k1_imu/dxl/202/202', source(3, 301, read_dhi_result='-3001'), now=10.4)
    assert cache.snapshot(10.4)[0]['level'] == 3
    cache.ingest_source('k1_imu/dxl/202/202', source(4, 401), now=10.5)
    assert cache.snapshot(10.5)[0]['level'] == 3
    cache.ingest_dynamic([('imu', ['Temperature'], [25.26])], stamp=2, now=10.6)
    assert cache.snapshot(10.6)[0]['level'] == 0
    cache.ingest_source('k1_imu/dxl/202/202', source(producer_session='new'), now=10.7)
    assert cache.snapshot(10.7)[0]['level'] == 3


@pytest.mark.parametrize('rows', [
    [('imu', ['Temperature'], [float('nan')])],
    [('imu', ['Temperature'], [327.68])],
    [('imu', ['Temperature'], [-327.69])],
    [('imu', ['Temperature'], [True])],
    [('dxl202', ['Temperature'], [25.0])],
    [('imu', ['Temperature', 'Temperature'], [25.0, 25.0])],
])
def test_invalid_imu_frame_does_not_preserve_live_cache(rows):
    cache = ImuTemperature('k1-test')
    prime(cache)
    assert not cache.ingest_dynamic(rows, stamp=2, now=10.4)
    assert cache.snapshot(10.4)[0]['level'] == 3


def test_motor_and_imu_identities_and_message_stamps_are_independent():
    imu, motors = ImuTemperature('k1-test'), MotorTemperatures('k1-test')
    prime(imu)
    assert not imu.ingest_source('k1_right_arm/dxl/1/1', source(), now=10.4)
    assert not motors.ingest_source('k1_imu/dxl/202/202', source(), now=10.4)
    assert len(motors.snapshot(10.4)) == 23
    assert imu.snapshot(10.4)[0]['level'] == 0
