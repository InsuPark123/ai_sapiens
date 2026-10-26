#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Woojin Wie

from launch import LaunchDescription
import os
import uuid

from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='false',
                              description='Whether to execute rviz2'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock hardware mirroring command.'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false',
                              description='Enable mock sensor commands.'),
        DeclareLaunchArgument('imu_temperature_enabled', default_value='true',
                              description='Read ICM42688 IMU temperature and publish cached diagnostics at 5 Hz.'),
        DeclareLaunchArgument('motor_temperatures_enabled', default_value='true',
                              description='Read motor temperatures and publish cached diagnostics at 5 Hz.'),
        DeclareLaunchArgument('model', default_value='k1_rev1',
                              description='Model name.'),
        DeclareLaunchArgument('diagnostics_enabled', default_value='true',
                              description='Enable DHI observations and independent health processes.'),
        DeclareLaunchArgument('diagnostics_response_evidence_enabled', default_value='true',
                              description='Capture failed Optimized read evidence for app localization.'),
        DeclareLaunchArgument('diagnostics_standard_aggregation', default_value='true',
                              description='Group /diagnostics using the standard K1 rev1 analyzer.'),
        DeclareLaunchArgument('diagnostics_hat_telemetry', default_value=LaunchConfiguration('diagnostics_enabled'),
                              description='Publish existing HAT/BMS memory at 5 Hz.'),
        DeclareLaunchArgument('restart', default_value='false',
                              description='Enable automatic E-stop recovery to initial bringup state.'),
        DeclareLaunchArgument('recovery_enabled', default_value=LaunchConfiguration('restart'),
                              description='Enable motor hardware lifecycle recovery.'),
        DeclareLaunchArgument('recovery_auto_rearm', default_value=LaunchConfiguration('restart'),
                              description='Permit recovery activation into the selected rearm mode.'),
        DeclareLaunchArgument('recovery_rearm_mode',
                              default_value=IfElseSubstitution(
                                  LaunchConfiguration('restart'), 'startup', 'damping'),
                              choices=['startup', 'damping']),
        DeclareLaunchArgument('recovery_damping_gain', default_value='3.0'),
        DeclareLaunchArgument('start_diagnostics_nodes', default_value='true',
                              description='Set false when a separate health launch already runs.'),
        DeclareLaunchArgument('robot_id', default_value=os.environ.get('ROBOT_ID', 'unassigned'),
                              description='Robot identity for app telemetry; set per robot.'),
        DeclareLaunchArgument('diagnostics_startup_directory',
                              default_value=os.environ.get('DHI_DIAGNOSTICS_STARTUP_DIRECTORY',
                                                          os.path.expanduser('~/.ros/dhi-diagnostics/startup'))),
        DeclareLaunchArgument('diagnostics_run_id', default_value=str(uuid.uuid4()),
                              description='Unique per bringup; reuse only to replay startup history.'),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    model = LaunchConfiguration('model')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ai_sapiens_description'),
                              'urdf',
                              'k1_rev1',
                              'k1.urdf.xacro']),
        ' ',
        'use_mock_hardware:=', use_mock_hardware,
        ' ',
        'mock_sensor_commands:=', mock_sensor_commands,
        ' ',
        'model:=', model,
        ' imu_temperature_enabled:=', LaunchConfiguration('imu_temperature_enabled'),
        ' motor_temperatures_enabled:=', LaunchConfiguration('motor_temperatures_enabled'),
        ' diagnostics_enabled:=', LaunchConfiguration('diagnostics_enabled'),
        ' diagnostics_response_evidence_enabled:=', LaunchConfiguration('diagnostics_response_evidence_enabled'),
        ' recovery_enabled:=', LaunchConfiguration('recovery_enabled'),
        ' recovery_auto_rearm:=', LaunchConfiguration('recovery_auto_rearm'),
        ' recovery_rearm_mode:=', LaunchConfiguration('recovery_rearm_mode'),
        ' recovery_damping_gain:=', LaunchConfiguration('recovery_damping_gain'),
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_bringup'), 'config',
        'k1_rev1', 'k1_rev1_controllers.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_description'), 'rviz', 'k1.rviz'
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config,
                    PathJoinSubstitution([FindPackageShare('ai_sapiens_diagnostics'), 'config', 'k1_hat_telemetry.yaml'])],
        output='both',
        additional_env={
            'DHI_DIAGNOSTICS_STARTUP_DIRECTORY': LaunchConfiguration('diagnostics_startup_directory'),
            'DHI_DIAGNOSTICS_RUN_ID': LaunchConfiguration('diagnostics_run_id'),
        },
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'imu_sensor_broadcaster',
        ],
        output='screen'
    )

    motor_temperature_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['motor_temperature_broadcaster'], output='screen',
        condition=IfCondition(LaunchConfiguration('motor_temperatures_enabled')),
    )
    imu_temperature_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['imu_temperature_broadcaster'], output='screen',
        condition=IfCondition(LaunchConfiguration('imu_temperature_enabled')),
    )
    motor_temperature_diagnostics = Node(
        package='ai_sapiens_diagnostics', executable='temperature_node',
        parameters=[{'robot_id': LaunchConfiguration('robot_id'),
                     'motor_temperatures_enabled': ParameterValue(LaunchConfiguration('motor_temperatures_enabled'), value_type=bool),
                     'imu_temperature_enabled': ParameterValue(LaunchConfiguration('imu_temperature_enabled'), value_type=bool)}],
        output='screen',
        condition=IfCondition(IfElseSubstitution(LaunchConfiguration('motor_temperatures_enabled'),
                                                'true', LaunchConfiguration('imu_temperature_enabled'))),
    )

    rc_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rc_broadcaster', 'group_rc_broadcaster'],
        output='screen'
    )

    joint_group_impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_impedance_controller'],
        output='screen'
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ai_sapiens_diagnostics'), 'launch', 'health.launch.py'])),
        launch_arguments={
            'robot_id': LaunchConfiguration('robot_id'),
            'startup_directory': LaunchConfiguration('diagnostics_startup_directory'),
            'startup_run_id': LaunchConfiguration('diagnostics_run_id'),
            'start_startup_observer': 'true',
            'start_standard_aggregator': LaunchConfiguration('diagnostics_standard_aggregation'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('diagnostics_enabled')),
    )

    recovery = Node(
        package='ai_sapiens_recovery', executable='recovery_node', output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('ai_sapiens_recovery'), 'config', 'k1.yaml']),
            {'execute': True,
             'auto_rearm': ParameterValue(LaunchConfiguration('recovery_auto_rearm'), value_type=bool),
             'rearm_mode': LaunchConfiguration('recovery_rearm_mode')}],
        condition=IfCondition(LaunchConfiguration('recovery_enabled')),
    )

    hat_telemetry = Node(package='controller_manager', executable='spawner',
                         arguments=['hat_state_broadcaster'], output='screen',
                         condition=IfCondition(LaunchConfiguration('diagnostics_hat_telemetry')))

    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=validate_response_evidence),
            OpaqueFunction(function=validate_recovery_options),
            OpaqueFunction(function=validate_motor_temperatures),
            GroupAction(actions=[diagnostics],
                        condition=IfCondition(LaunchConfiguration('start_diagnostics_nodes'))),
            recovery,
            hat_telemetry,
            motor_temperature_spawner,
            imu_temperature_spawner,
            motor_temperature_diagnostics,
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            rc_broadcaster_spawner,
            joint_group_impedance_controller_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            # rviz_node,
        ]
    )


def validate_response_evidence(context):
    # Explicitly disabling capture preserves compatibility with older installations.
    value = context.launch_configurations.get('diagnostics_response_evidence_enabled', 'false')
    if value.lower() in ('false', '0'):
        return []
    from ai_sapiens_diagnostics.preflight import validate_response_evidence_launch
    return validate_response_evidence_launch(context)


def validate_recovery_options(context):
    enabled = IfCondition(LaunchConfiguration('recovery_enabled')).evaluate(context)
    rearm = IfCondition(LaunchConfiguration('recovery_auto_rearm')).evaluate(context)
    if rearm and not enabled:
        raise RuntimeError('recovery_auto_rearm requires recovery_enabled:=true')
    if enabled and not IfCondition(LaunchConfiguration('diagnostics_enabled')).evaluate(context):
        raise RuntimeError('recovery_enabled requires diagnostics_enabled:=true')
    if enabled and IfCondition(LaunchConfiguration('use_mock_hardware')).evaluate(context):
        raise RuntimeError('K1 recovery requires real hardware; use the recovery mock tests instead')
    return []


def validate_motor_temperatures(context):
    enabled = IfCondition(LaunchConfiguration('motor_temperatures_enabled')).evaluate(context)
    diagnostics = IfCondition(LaunchConfiguration('diagnostics_enabled')).evaluate(context)
    if enabled and not diagnostics:
        raise RuntimeError('motor_temperatures_enabled requires diagnostics_enabled:=true for sample freshness.')
    imu_enabled = IfCondition(LaunchConfiguration('imu_temperature_enabled')).evaluate(context)
    if imu_enabled and not diagnostics:
        raise RuntimeError('imu_temperature_enabled requires diagnostics_enabled:=true for sample freshness.')
    return []
