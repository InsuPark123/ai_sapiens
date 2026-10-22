# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Validate the expanded inventory without instantiating a hardware plugin."""
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml
import pytest

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / 'ai_sapiens_description/urdf/k1_rev1/k1.urdf.xacro'


def expanded(mock=False, enabled=True, evidence=False):
    return ET.fromstring(xacro.process_file(str(DESCRIPTION), mappings={
        'use_mock_hardware': str(mock).lower(), 'diagnostics_enabled': str(enabled).lower(),
        'diagnostics_response_evidence_enabled': str(evidence).lower(),
    }).toxml())


def test_real_inventory_matches_app_contract():
    root = expanded()
    inventory = set()
    for system in root.findall('ros2_control'):
        hardware = system.find('hardware')
        params = {p.attrib['name']: p.text for p in hardware.findall('param')}
        assert params['diagnostics_enabled'].lower() == 'true'
        joints = {j.attrib['name'] for j in system.findall('joint')}
        for gpio in system.findall('gpio'):
            fields = {p.attrib['name']: p.text for p in gpio.findall('param')}
            if fields['type'] == 'dxl':
                assert fields['diagnostic_joint'] in joints
            dxl_id = fields['ID']
            comm_id = fields.get('comm_id', dxl_id)
            inventory.add(f"dhi/{system.attrib['name']}/dxl/{comm_id}/{dxl_id}")
    config = yaml.safe_load((ROOT / 'ai_sapiens_diagnostics/config/k1.yaml').read_text())
    assert inventory == set(config['ai_sapiens_health']['ros__parameters']['expected_components'])
    assert len(inventory) == 25


def test_mock_description_never_loads_dhi():
    root = expanded(mock=True)
    plugins = [p.text for p in root.findall('./ros2_control/hardware/plugin')]
    assert len(plugins) == 6
    assert set(plugins) == {'mock_components/GenericSystem'}


def test_standard_aggregator_inventory_matches_description():
    root = expanded()
    expected = set()
    for system in root.findall('ros2_control'):
        name = system.attrib['name']
        expected.add(f'{name}/bus')
        for gpio in system.findall('gpio'):
            fields = {p.attrib['name']: p.text for p in gpio.findall('param')}
            dxl_id = fields['ID']
            expected.add(f"{name}/dxl/{fields.get('comm_id', dxl_id)}/{dxl_id}")
    config = yaml.safe_load((ROOT / 'ai_sapiens_diagnostics/config/k1_aggregator.yaml').read_text())
    analyzers = config['analyzers']['ros__parameters']['analyzers']
    actual = []
    for analyzer in analyzers.values():
        inventory = analyzer.get('expected', [])
        if inventory:
            assert analyzer['num_items'] == len(inventory)
            assert all(item.startswith(tuple(analyzer['startswith'])) for item in inventory)
            actual.extend(inventory)
    assert len(actual) == len(set(actual)) == 31
    assert set(actual) == expected


def test_diagnostics_can_be_disabled_without_changing_driver():
    root = expanded(enabled=False)
    for system in root.findall('ros2_control'):
        hardware = system.find('hardware')
        assert hardware.find('plugin').text == 'dynamixel_hardware_interface/DynamixelHardware'
        params = {p.attrib['name']: p.text for p in hardware.findall('param')}
        assert params['diagnostics_enabled'].lower() == 'false'
        # Older DHI treats unknown GPIO parameters as control-table writes.
        assert not system.findall("./gpio/param[@name='diagnostic_joint']")


@pytest.mark.parametrize('evidence', [False, True])
def test_response_capture_flag_reaches_all_six_components(evidence):
    root = expanded(evidence=evidence)
    systems = root.findall('ros2_control')
    assert len(systems) == 6
    for system in systems:
        params = {p.attrib['name']: p.text for p in system.findall('./hardware/param')}
        assert params['diagnostics_response_evidence_enabled'] == str(evidence).lower()
    # Enabling capture must not add a register, ID, joint, or a read/write field.
    other = expanded(evidence=not evidence)
    for description in (root, other):
        for hardware in description.findall('./ros2_control/hardware'):
            hardware.remove(hardware.find("param[@name='diagnostics_response_evidence_enabled']"))
    assert ET.tostring(root) == ET.tostring(other)
