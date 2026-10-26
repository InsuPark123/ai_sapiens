# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Group existing diagnostics; this launch never starts robot hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_diagnostics'), 'config', 'k1_aggregator.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('aggregator_config', default_value=config,
                              description='K1 rev1 inventory; override for other robots.'),
        DeclareLaunchArgument('diagnostics_input', default_value='/diagnostics'),
        DeclareLaunchArgument('diagnostics_output', default_value='/diagnostics_agg'),
        DeclareLaunchArgument('diagnostics_toplevel_output',
                              default_value='/diagnostics_toplevel_state'),
        Node(package='diagnostic_aggregator', executable='aggregator_node', name='analyzers',
             parameters=[LaunchConfiguration('aggregator_config')],
             remappings=[
                 ('/diagnostics', LaunchConfiguration('diagnostics_input')),
                 ('/diagnostics_agg', LaunchConfiguration('diagnostics_output')),
                 ('/diagnostics_toplevel_state',
                  LaunchConfiguration('diagnostics_toplevel_output')),
             ], output='screen'),
    ])
