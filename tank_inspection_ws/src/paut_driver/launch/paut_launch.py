#!/usr/bin/env python3
"""Launch the PAUT UDP receive driver in isolation (no recorder)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('paut_driver')
    params = os.path.join(pkg, 'config', 'paut_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('sensor_id', default_value='UNASSIGNED'),
        DeclareLaunchArgument('calibration_id', default_value='UNASSIGNED'),
        DeclareLaunchArgument('frame_id', default_value='paut_probe_link'),
        DeclareLaunchArgument('sample_encoding',
            default_value='signed_int32_host_endian_channel_major'),
        DeclareLaunchArgument('sampling_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('gain_db', default_value='-1.0'),
        DeclareLaunchArgument('sound_velocity_m_s', default_value='-1.0'),
        Node(
            package='paut_driver', executable='paut_driver_node',
            name='paut_driver_node', output='screen',
            parameters=[params, {
                'port': LaunchConfiguration('port'),
                'sensor_id': LaunchConfiguration('sensor_id'),
                'calibration_id': LaunchConfiguration('calibration_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'sample_encoding': LaunchConfiguration('sample_encoding'),
                'sampling_rate_hz': LaunchConfiguration('sampling_rate_hz'),
                'gain_db': LaunchConfiguration('gain_db'),
                'sound_velocity_m_s': LaunchConfiguration('sound_velocity_m_s'),
            }],
        ),
    ])
