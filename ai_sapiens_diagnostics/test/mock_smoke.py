#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Offline end-to-end test: real DHI reporter/journal -> ROS -> app snapshot.

No controller manager, robot port, policy, or actuation is started. The startup
fixture deliberately fails on /dev/dhi_diagnostics_nonexistent_test_port.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ.update(ROS_DOMAIN_ID='188', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                  ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
os.environ.pop('ROS_STATIC_PEERS', None)

from ament_index_python.packages import (  # noqa: E402
    get_package_prefix, get_package_share_directory)
import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402


def main():
    dhi = Path(get_package_prefix('dynamixel_hardware_interface'))
    mock = str(dhi / 'lib/dynamixel_hardware_interface/diagnostics_1khz_mock')
    config = str(Path(get_package_share_directory('ai_sapiens_diagnostics')) / 'config/mock.yaml')
    children, seen = [], set()
    latest = {}
    unaffected = False
    rclpy.init()
    node = rclpy.create_node('diagnostics_integration_observer')

    def receive(message):
        nonlocal latest, unaffected
        latest = json.loads(message.data)
        components = {c['name']: c for c in latest['components']}
        faulted = components['dhi/mock/dxl/200/24']
        other = components['dhi/mock/dxl/200/21']
        seen.add(faulted['reason'])
        if faulted['reason'] == 'DEVICE_REPORTED_FAULT':
            unaffected |= other['health'] == 'OK'
            assert faulted['evidence']['error_code'] == '4'
        if faulted['reason'] == 'GROUP_READ_FAILED':
            assert other['health'] == faulted['health'] == 'UNKNOWN'
            assert faulted['evidence']['rx_reason'] == 'RX_TIMEOUT'
        if faulted['reason'] == 'GROUP_WRITE_FAILED':
            assert faulted['evidence']['tx_reason'] == 'TX_FAIL'
            assert faulted['evidence']['command_applied'] == 'unknown'

    subscription = node.create_subscription(String, '/ai_sapiens/health', receive, 10)
    with tempfile.TemporaryDirectory(prefix='dhi-app-smoke-') as directory:
        with tempfile.TemporaryFile(mode='w+') as log:
            try:
                # Writer exits before the observer exists: DDS durability alone
                # cannot supply this failure; the file journal must survive.
                env = dict(os.environ, DHI_DIAGNOSTICS_STARTUP_DIRECTORY=directory,
                           DHI_DIAGNOSTICS_RUN_ID='offline-run')
                startup_fixture = (dhi / 'lib/dynamixel_hardware_interface' /
                                   'diagnostics_startup_failure')
                subprocess.run([str(startup_fixture)],
                               env=env, check=True,
                               timeout=10, stdout=log, stderr=log)
                assert list(Path(directory).rglob('*.cdr')), 'Missing startup journal'
                launch = subprocess.Popen([
                    'ros2', 'launch', 'ai_sapiens_diagnostics', 'health.launch.py',
                    'config:=' + config, 'robot_id:=offline',
                    'startup_directory:=' + directory, 'startup_run_id:=offline-run',
                    'start_startup_observer:=true'],
                    stdout=log, stderr=log, start_new_session=True)
                children.append(launch)
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline and not latest.get('startup', {}).get('records'):
                    assert launch.poll() is None, 'Health launch exited early'
                    rclpy.spin_once(node, timeout_sec=0.1)
                assert latest['health'] == 'UNKNOWN'
                records = latest['startup']['records']
                failure = next(r for r in records if r['name'] == 'dhi_startup/startup_test')
                assert failure['level'] == 2 and failure['evidence']['phase'] == 'FAILED'
                assert failure['evidence']['stage'] == 'port'
                # A late DDS match must not skip the one-second fault interval.
                # This file gates the fixture before its measured 1 kHz loop.
                ready = Path(directory) / 'runtime-consumer-ready'
                producer_env = dict(os.environ, DHI_DIAGNOSTICS_MOCK_READY_FILE=str(ready))
                producer = subprocess.Popen([mock, 'mixed'], stdout=log, stderr=log,
                                            start_new_session=True, env=producer_env)
                children.append(producer)
                deadline = time.monotonic() + 8
                connected = False
                while time.monotonic() < deadline:
                    assert producer.poll() is None, 'Producer exited before readiness handshake'
                    rclpy.spin_once(node, timeout_sec=0.05)
                    connected = any(
                        c.get('evidence', {}).get('producer_session')
                        for c in latest.get('components', []))
                    if connected:
                        break
                assert connected, 'App did not receive the mock diagnostic stream'
                ready.touch()
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    assert launch.poll() is None, 'Health launch exited during runtime'
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert producer.poll() == 0, 'Producer did not complete successfully'
                assert {'OBSERVED_OK', 'DEVICE_REPORTED_FAULT', 'GROUP_READ_FAILED',
                        'GROUP_WRITE_FAILED', 'PRODUCER_EXPIRED'} <= seen, sorted(seen)
                assert unaffected
                assert latest['health'] == 'UNKNOWN'
                faulted = next(c for c in latest['components']
                               if c['name'] == 'dhi/mock/dxl/200/24')
                assert faulted['evidence']['last_error_code'] == '4'
                assert faulted['evidence']['last_read_error_sdk'] == '-3001'
                assert faulted['evidence']['last_write_error_sdk'] == '-1001'
                assert latest['coverage']['restart_authorized'] is False
                print(json.dumps(dict(passed=True, reasons=sorted(seen),
                                      startup_failure_survived_writer_exit=True,
                                      unaffected_id_stayed_healthy=unaffected,
                                      final_health=latest['health'])))
                # Preserve the producer's allocation/timing result in the test log.
                log.seek(0)
                for line in log:
                    if line.startswith('{"mode":'):
                        print(line.strip())
            except Exception:
                log.seek(0)
                print(log.read())
                raise
            finally:
                for child in reversed(children):
                    if child.poll() is None:
                        os.killpg(child.pid, signal.SIGINT)
                        try:
                            child.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(child.pid, signal.SIGKILL)
                            child.wait()
                node.destroy_subscription(subscription)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
