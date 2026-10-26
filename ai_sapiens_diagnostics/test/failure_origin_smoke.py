#!/usr/bin/env python3
"""Real ROS message replay; never starts controller_manager or accesses a DXL port."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

# Set before importing rclpy; do not join a robot's production ROS domain.
os.environ.update(ROS_DOMAIN_ID='188', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                  ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_LOCALHOST_ONLY='1')
os.environ.pop('ROS_STATIC_PEERS', None)
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from std_msgs.msg import String

NAME = 'k1_left_leg/dxl/200/21'
CANONICAL = 'dhi/' + NAME


def fields(session, sequence):
    return dict(diagnostics_schema='2', producer_session=session,
                publication_sequence=str(sequence), phase='FAILED', id='21', comm_id='200',
                sample_count='1', sample_age_ms='0', stale_after_ms='5000',
                sample_received='true', sample_fresh='false', error_code_available='true',
                hardware_error_available='false', estop_available='false', invalid_value='false',
                error_code='0', hardware_error_status='0', estop_active='0',
                read_dhi_result='-7', write_dhi_result='0', read_sdk_result='-3002',
                write_sdk_result='0', read_failures='1', write_failures='0', fault_changes='0',
                last_error_code='0', last_hardware_error_status='0', last_estop_active='0')


def status(name, level, data):
    return DiagnosticStatus(name=name, hardware_id='SYNTHETIC_NO_HARDWARE',
                            level=bytes([level]), message='Synthetic failure replay',
                            values=[KeyValue(key=k, value=v) for k, v in data.items()])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    executable = Path(get_package_prefix('ai_sapiens_diagnostics')) / 'lib/ai_sapiens_diagnostics/health_node'
    rclpy.init()
    node = rclpy.create_node('failure_origin_replay')
    latest = {}
    results = []

    def receive(message):
        nonlocal latest
        latest = json.loads(message.data)

    subscription = node.create_subscription(String, '/ai_sapiens/health', receive, 10)
    publisher = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
    command = [str(executable), '--ros-args', '-p', 'robot_id:=synthetic', '-p',
               'expected_components:=[' + CANONICAL + ']', '-p', 'publish_rate_hz:=20.0']
    with (output / 'health-node.log').open('w') as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 10
            while publisher.get_subscription_count() == 0:
                assert child.poll() is None, 'health node exited before discovery'
                assert time.monotonic() < deadline, 'ROS discovery timeout'
                rclpy.spin_once(node, timeout_sec=0.05)
            cases = [('running', 'OBSERVED_OK', 'OK', None),
                     ('runtime_reversed', 'GROUP_READ_FAILED', 'UNKNOWN', 'runtime'),
                     ('startup', 'INITIALIZATION_FAILED', 'ERROR', 'startup'),
                     ('missing_bus', 'HARDWARE_FAILURE_ORIGIN_UNKNOWN', 'UNKNOWN', 'unknown'),
                     ('wrong_sequence', 'HARDWARE_FAILURE_ORIGIN_UNKNOWN', 'UNKNOWN', 'unknown'),
                     ('duplicate_bus', 'HARDWARE_FAILURE_ORIGIN_UNKNOWN', 'UNKNOWN', 'unknown'),
                     ('duplicate_key', 'HARDWARE_FAILURE_ORIGIN_UNKNOWN', 'UNKNOWN', 'unknown'),
                     ('runtime_write', 'GROUP_WRITE_FAILED', 'UNKNOWN', 'runtime'),
                     ('runtime_without_io_error', 'RUNTIME_HARDWARE_FAILED', 'UNKNOWN', 'runtime'),
                     ('running_after_failure', 'OBSERVED_OK', 'OK', None)]
            for case, reason, health, origin in cases:
                latest = {}
                matched = None
                sequence = 0
                deadline = time.monotonic() + 4
                while time.monotonic() < deadline:
                    assert child.poll() is None, 'health node exited during replay'
                    sequence += 1
                    device_fields = fields(case, sequence)
                    if origin is None:
                        device_fields.update(phase='RUNNING', sample_fresh='true',
                                             read_dhi_result='0', read_sdk_result='0',
                                             read_failures='0')
                    if case in ('runtime_write', 'runtime_without_io_error'):
                        device_fields.update(read_dhi_result='0', read_sdk_result='',
                                             write_dhi_result='-6' if case == 'runtime_write' else '0')
                    bus_fields = dict(device_fields, failure_origin='startup' if case == 'startup' else 'runtime')
                    if case == 'wrong_sequence':
                        bus_fields['publication_sequence'] = str(sequence + 1)
                    bus = status('k1_left_leg/bus', 2, bus_fields)
                    device = status(NAME, 0 if origin is None else
                                    2 if case == 'startup' else 3, device_fields)
                    rows = [device, bus]  # Deliberately put the device before the bus.
                    if case == 'missing_bus':
                        rows = [device]
                    elif case == 'duplicate_bus':
                        rows.append(status('k1_left_leg/bus', 2, dict(bus_fields, failure_origin='startup')))
                    elif case == 'duplicate_key':
                        bus.values.append(KeyValue(key='failure_origin', value='startup'))
                    message = DiagnosticArray(status=rows)
                    message.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    candidate = next((c for c in latest.get('components', [])
                                      if c.get('producer_session') == case), None)
                    if candidate is not None:
                        matched = candidate
                        assert candidate['reason'] == reason, (case, candidate)
                        assert candidate['health'] == health, (case, candidate)
                        assert candidate['quality'] == ('FRESH' if origin is None else 'STALE'), (case, candidate)
                        if origin is not None:
                            assert candidate['evidence']['failure_origin'] == origin, (case, candidate)
                        assert latest['coverage']['restart_authorized'] is False
                        results.append(dict(case=case, snapshot=latest))
                        print('PASS', case, reason, flush=True)
                        break
                assert matched is not None, 'No matching health snapshot: ' + case
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=5)
                    raise RuntimeError('Owned health test process needed forced shutdown')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            (output / 'results.json').write_text(json.dumps(dict(command=command, cases=results,
                scope='isolated domain 188; synthetic packets only; no CM/DHI/SDK/motor commands'), indent=2))


if __name__ == '__main__':
    main()
