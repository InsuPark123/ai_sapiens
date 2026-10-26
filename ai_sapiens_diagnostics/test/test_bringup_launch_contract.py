# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Check the normal K1 launch contract without starting hardware processes."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.utilities import perform_substitutions
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import pytest

from ai_sapiens_diagnostics import preflight


LAUNCH_FILE = (Path(__file__).resolve().parents[2] /
               'ai_sapiens_bringup/launch/k1.launch.py')


def normal_launch():
    spec = importlib.util.spec_from_file_location('normal_k1_bringup', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, module.generate_launch_description()


def test_normal_launch_enables_diagnostics_but_keeps_recovery_opt_in():
    _, description = normal_launch()
    entities = description.entities
    context = LaunchContext()
    declarations = {a.name: a for a in entities if isinstance(a, DeclareLaunchArgument)}
    for action in declarations.values():
        action.execute(context)
    for name in ('diagnostics_enabled', 'diagnostics_response_evidence_enabled',
                 'diagnostics_standard_aggregation', 'motor_temperatures_enabled', 'imu_temperature_enabled'):
        assert perform_substitutions(context, declarations[name].default_value) == 'true'
    for name in ('recovery_enabled', 'recovery_auto_rearm'):
        assert perform_substitutions(context, declarations[name].default_value) == 'false'
    actions = [a for a in entities if not isinstance(a, DeclareLaunchArgument)]
    assert isinstance(actions[0], OpaqueFunction)
    assert isinstance(actions[1], OpaqueFunction)
    assert isinstance(actions[2], OpaqueFunction)
    assert isinstance(actions[3], GroupAction)
    nodes = [a for a in actions if isinstance(a, Node)]
    active = [a for a in nodes if a.condition is None or a.condition.evaluate(context)]
    assert len(active) == 9
    recovery = next(a for a in nodes if a.node_package == 'ai_sapiens_recovery')
    assert not recovery.condition.evaluate(context)


def test_production_launch_has_no_robot_specific_installation_path(monkeypatch):
    monkeypatch.delenv('ROBOT_ID', raising=False)
    monkeypatch.delenv('DHI_DIAGNOSTICS_STARTUP_DIRECTORY', raising=False)
    _, description = normal_launch()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['robot_id'] == 'unassigned'
    directory = Path(context.launch_configurations['diagnostics_startup_directory'])
    assert directory == Path.home() / '.ros/dhi-diagnostics/startup'
    source = LAUNCH_FILE.read_text()
    for marker in ('k1-18', 'k1-23', '/workspace/', 'setup_prepared', 'setup_k1.sh'):
        assert marker not in source


def test_existing_group_radio_controller_is_still_spawned():
    _, description = normal_launch()
    nodes = [a for a in description.entities if isinstance(a, Node)]
    rc = [a for a in nodes if a.node_package == 'controller_manager'
          and any(isinstance(arg, TextSubstitution) and arg.text == 'group_rc_broadcaster'
                  for group in a.cmd for arg in group)]
    assert len(rc) == 1


def test_normal_launch_off_does_not_import_or_probe_driver(monkeypatch):
    module, _ = normal_launch()
    probe = Mock(side_effect=AssertionError('Explicit capture OFF must not probe capture'))
    monkeypatch.setattr(preflight, 'validate_response_evidence_launch', probe)
    assert module.validate_response_evidence(SimpleNamespace(launch_configurations={
        'diagnostics_response_evidence_enabled': 'false'})) == []
    probe.assert_not_called()


def test_normal_launch_rejects_capture_without_diagnostics():
    module, _ = normal_launch()
    with pytest.raises(RuntimeError, match='requires diagnostics_enabled'):
        module.validate_response_evidence(SimpleNamespace(launch_configurations={
            'diagnostics_response_evidence_enabled': 'true',
            'diagnostics_enabled': 'false'}))


def test_normal_launch_mock_never_probes_physical_driver(monkeypatch):
    module, _ = normal_launch()
    probe = Mock(side_effect=AssertionError('Mock launch must not inspect DHI'))
    monkeypatch.setattr(preflight, 'check_response_evidence', probe)
    assert module.validate_response_evidence(SimpleNamespace(launch_configurations={
        'diagnostics_response_evidence_enabled': 'true', 'diagnostics_enabled': 'true',
        'use_mock_hardware': 'true'})) == []
    probe.assert_not_called()


def test_normal_launch_contains_no_timing_observer_injection():
    # Measurement helpers belong to test launch copies, never the normal launch.
    source = LAUNCH_FILE.read_text()
    for marker in ('BENCH_CM_', 'LD_PRELOAD', 'instrumented-k1', 'run_cable_trial'):
        assert marker not in source


def test_default_launch_checks_selected_capture_capability(monkeypatch):
    module, description = normal_launch()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    probe = Mock(return_value=[])
    monkeypatch.setattr(preflight, 'validate_response_evidence_launch', probe)
    assert module.validate_response_evidence(context) == []
    probe.assert_called_once_with(context)
    assert context.launch_configurations['diagnostics_response_evidence_enabled'] == 'true'


@pytest.mark.parametrize('overrides,enabled,rearm,mode', [
    ({}, False, False, 'damping'),
    ({'restart': 'true'}, True, True, 'startup'),
    ({'restart': 'true', 'recovery_rearm_mode': 'damping'}, True, True, 'damping'),
    ({'recovery_enabled': 'true'}, True, False, 'damping'),
    ({'recovery_enabled': 'true', 'recovery_auto_rearm': 'true',
      'recovery_rearm_mode': 'startup'}, True, True, 'startup'),
    ({'restart': 'true', 'recovery_auto_rearm': 'false'}, True, False, 'startup'),
])
def test_one_normal_launch_configures_hardware_and_supervisor_together(overrides, enabled, rearm, mode):
    from launch_ros.utilities import evaluate_parameters
    module, description = normal_launch()
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert module.validate_recovery_options(context) == []
    recovery = next(a for a in description.entities
                    if isinstance(a, Node) and a.node_package == 'ai_sapiens_recovery')
    assert recovery.condition.evaluate(context) is enabled
    assert context.launch_configurations['recovery_auto_rearm'] == str(rearm).lower()
    assert context.launch_configurations['recovery_rearm_mode'] == mode
    # Resolve actual ROS typed overrides without launching any process or opening a port.
    override = evaluate_parameters(context, recovery._Node__parameters[1:])[0]
    assert override == {'execute': True, 'auto_rearm': rearm, 'rearm_mode': mode}


@pytest.mark.parametrize('overrides,match', [
    ({'recovery_auto_rearm': 'true', 'recovery_enabled': 'false'}, 'requires recovery_enabled'),
    ({'restart': 'true', 'diagnostics_enabled': 'false'}, 'requires diagnostics_enabled'),
    ({'restart': 'true', 'use_mock_hardware': 'true'}, 'requires real hardware'),
])
def test_inconsistent_recovery_flags_fail_before_hardware_actions(overrides, match):
    module, description = normal_launch()
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    with pytest.raises(RuntimeError, match=match):
        module.validate_recovery_options(context)


def test_legacy_recovery_launch_does_not_start_a_second_supervisor():
    path = LAUNCH_FILE.parents[2] / 'ai_sapiens_recovery/launch/k1_recovery_test.launch.py'
    spec = importlib.util.spec_from_file_location('legacy_recovery_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert not any(isinstance(a, Node) for a in module.generate_launch_description().entities)


def test_site_identity_and_persistent_journal_can_be_set_without_editing_launch(monkeypatch):
    monkeypatch.setenv('ROBOT_ID', 'k1-42')
    monkeypatch.setenv('DHI_DIAGNOSTICS_STARTUP_DIRECTORY', '/var/diagnostics/startup')
    _, description = normal_launch()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['robot_id'] == 'k1-42'
    assert context.launch_configurations['diagnostics_startup_directory'] == '/var/diagnostics/startup'
    # Explicit launch arguments still win over the site defaults.
    context = LaunchContext()
    context.launch_configurations['robot_id'] = 'k1-43'
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['robot_id'] == 'k1-43'


@pytest.mark.parametrize('enabled', ['false', 'true'])
@pytest.mark.parametrize('imu_enabled', ['false', 'true'])
def test_temperature_producer_and_broadcaster_share_opt_in(enabled, imu_enabled):
    module, description = normal_launch()
    context = LaunchContext()
    context.launch_configurations['motor_temperatures_enabled'] = enabled
    context.launch_configurations['imu_temperature_enabled'] = imu_enabled
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert module.validate_motor_temperatures(context) == []
    nodes = [a for a in description.entities if isinstance(a, Node)]
    producer = next(a for a in nodes if a.node_package == 'ai_sapiens_diagnostics')
    spawner = next(a for a in nodes if a.node_package == 'controller_manager'
                   and any(isinstance(arg, TextSubstitution) and arg.text == 'motor_temperature_broadcaster'
                           for group in a.cmd for arg in group))
    assert producer.condition.evaluate(context) is (enabled == 'true' or imu_enabled == 'true')
    imu_spawner = next(a for a in nodes if a.node_package == 'controller_manager'
                       and any(isinstance(arg, TextSubstitution) and arg.text == 'imu_temperature_broadcaster'
                               for group in a.cmd for arg in group))
    assert imu_spawner.condition.evaluate(context) is (imu_enabled == 'true')
    from launch_ros.utilities import evaluate_parameters
    params = evaluate_parameters(context, producer._Node__parameters)[0]
    assert params['motor_temperatures_enabled'] is (enabled == 'true')
    assert params['imu_temperature_enabled'] is (imu_enabled == 'true')
    assert spawner.condition.evaluate(context) is (enabled == 'true')


def test_temperature_without_dhi_freshness_is_rejected():
    module, description = normal_launch()
    context = LaunchContext()
    context.launch_configurations.update(motor_temperatures_enabled='true', diagnostics_enabled='false')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    with pytest.raises(RuntimeError, match='requires diagnostics_enabled'):
        module.validate_motor_temperatures(context)


def test_imu_temperature_requires_diagnostics_even_without_motor_temperatures():
    module, description = normal_launch()
    context = LaunchContext()
    context.launch_configurations.update(motor_temperatures_enabled='false',
                                        imu_temperature_enabled='true', diagnostics_enabled='false')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    with pytest.raises(RuntimeError, match='imu_temperature_enabled requires diagnostics_enabled'):
        module.validate_motor_temperatures(context)
