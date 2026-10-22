# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_diagnostics'), 'config', 'k1.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('robot_id', default_value='unassigned'),
        DeclareLaunchArgument('startup_run_id', default_value=''),
        DeclareLaunchArgument('startup_directory', default_value=''),
        DeclareLaunchArgument('start_startup_observer', default_value='false'),
        DeclareLaunchArgument('start_standard_aggregator', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ai_sapiens_diagnostics'), 'launch',
                'standard_aggregation.launch.py'])),
            condition=IfCondition(LaunchConfiguration('start_standard_aggregator'))),
        Node(package='dynamixel_hardware_interface', executable='dhi_startup_diagnostics',
             parameters=[{
                 'journal_directory': ParameterValue(
                     LaunchConfiguration('startup_directory'), value_type=str),
                 'run_id': ParameterValue(LaunchConfiguration('startup_run_id'), value_type=str),
             }], condition=IfCondition(LaunchConfiguration('start_startup_observer')),
             output='screen'),
        Node(package='ai_sapiens_diagnostics', executable='health_node',
             parameters=[LaunchConfiguration('config'),
                         {'robot_id': ParameterValue(
                             LaunchConfiguration('robot_id'), value_type=str),
                          'startup_run_id': ParameterValue(
                              LaunchConfiguration('startup_run_id'), value_type=str)}],
             output='screen'),
    ])
