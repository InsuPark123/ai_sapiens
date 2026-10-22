"""Opt-in independent H5 diagnostics; not included by the robot bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hat_host', default_value='192.168.200.200'),
        DeclareLaunchArgument('hat_service_id', default_value='200'),
        DeclareLaunchArgument('hat_output', default_value='~/.ros/hat-evidence'),
        Node(package='ai_sapiens_diagnostics', executable='hat_collector',
             name='hat_diagnostic_collector', output='screen', parameters=[{
                 'host': LaunchConfiguration('hat_host'),
                 'service_id': ParameterValue(LaunchConfiguration('hat_service_id'), value_type=int),
                 'output_directory': LaunchConfiguration('hat_output'),
             }]),
    ])
