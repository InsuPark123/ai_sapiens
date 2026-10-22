"""Only publish interfaces already present in the HAT read group."""
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from test_bringup_launch_contract import normal_launch

ROOT=Path(__file__).resolve().parents[2]

def test_read_only_standard_broadcaster_uses_existing_sensor_interfaces():
    config=yaml.safe_load((ROOT/'ai_sapiens_diagnostics/config/k1_hat_telemetry.yaml').read_text())['/**']
    assert config['controller_manager']['ros__parameters']['hat_state_broadcaster']['type']=='joint_state_broadcaster/JointStateBroadcaster'
    params=config['hat_state_broadcaster']['ros__parameters']
    assert params['update_rate']==5 and params['joints']==['hat']
    assert params['use_local_topics'] and params['publish_dynamic_joint_states']
    assert not params['use_urdf_to_filter']
    xml=ET.fromstring(xacro.process_file(str(ROOT/'ai_sapiens_description/urdf/k1_rev1/k1.urdf.xacro')).toxml())
    sensor=xml.find("./ros2_control[@name='k1_hat']/sensor[@name='hat']")
    fields={s.attrib['name'] for s in sensor.findall('state_interface')}
    assert set(params['interfaces'])<=fields

def test_telemetry_default_follows_diagnostics_and_can_be_disabled():
    _,description=normal_launch()
    context=LaunchContext()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    assert context.launch_configurations['diagnostics_hat_telemetry']=='true'
    context=LaunchContext();context.launch_configurations['diagnostics_enabled']='false'
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    assert context.launch_configurations['diagnostics_hat_telemetry']=='false'
