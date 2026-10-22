"""Explicit K1 experiment: recover hardware/controllers to initial bringup behavior."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='unassigned'),
        DeclareLaunchArgument('diagnostics_standard_aggregation', default_value='false'),
        DeclareLaunchArgument('recovery_rearm_mode', default_value='startup'),
        DeclareLaunchArgument('recovery_damping_gain', default_value='3.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ai_sapiens_bringup'), 'launch', 'k1.launch.py'])),
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'diagnostics_enabled': 'true', 'recovery_enabled': 'true',
                'diagnostics_standard_aggregation':
                    LaunchConfiguration('diagnostics_standard_aggregation'),
                'recovery_auto_rearm': 'true',
                'recovery_rearm_mode': LaunchConfiguration('recovery_rearm_mode'),
                'recovery_damping_gain': LaunchConfiguration('recovery_damping_gain'),
            }.items()),
    ])
