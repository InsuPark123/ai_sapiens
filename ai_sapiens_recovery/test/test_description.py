"""Expand the actual K1 description; never instantiate hardware in this test."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import xacro

PATH = Path(__file__).resolve().parents[2] / 'ai_sapiens_description/urdf/k1_rev1/k1.urdf.xacro'


@pytest.mark.parametrize('enabled', ['false', 'true'])
@pytest.mark.parametrize('mode', ['damping', 'startup'])
def test_recovery_is_opt_in_and_only_applies_to_motor_buses(enabled, mode):
    root = ET.fromstring(xacro.process_file(str(PATH), mappings={
        'recovery_enabled': enabled, 'recovery_auto_rearm': enabled,
        'recovery_damping_gain': '3.0',
        'recovery_rearm_mode': mode,
    }).toxml())
    motors = {'k1_left_arm', 'k1_right_arm', 'k1_left_leg', 'k1_right_leg'}
    assert len(root.findall('ros2_control')) == 6
    for system in root.findall('ros2_control'):
        params = {x.attrib['name']: x.text for x in system.findall('./hardware/param')}
        if system.attrib['name'] in motors:
            assert params['recovery_enabled'] == enabled
            assert params['recovery_auto_rearm'] == enabled
            assert params['recovery_damping_gain'] == '3.0'
            assert params['recovery_rearm_mode'] == mode
        else:
            assert 'recovery_auto_rearm' not in params
