#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('eddy_driver')
    params = os.path.join(pkg, 'config', 'eddy_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='4000000'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('settings_query_rate', default_value='1.0'),
        DeclareLaunchArgument('sensor_id', default_value='UNASSIGNED'),
        DeclareLaunchArgument('calibration_id', default_value='UNASSIGNED'),
        DeclareLaunchArgument('frame_id', default_value='eddy_probe_link'),
        DeclareLaunchArgument('sampling_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('gain_db', default_value='-1.0'),
        DeclareLaunchArgument('lift_off_mm', default_value='-1.0'),
        Node(
            package='eddy_driver', executable='eddy_driver_node',
            name='eddy_driver_node', output='screen',
            parameters=[params, {
                'device': LaunchConfiguration('device'),
                'baud': LaunchConfiguration('baud'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'settings_query_rate': LaunchConfiguration('settings_query_rate'),
                'sensor_id': LaunchConfiguration('sensor_id'),
                'calibration_id': LaunchConfiguration('calibration_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'sampling_rate_hz': LaunchConfiguration('sampling_rate_hz'),
                'gain_db': LaunchConfiguration('gain_db'),
                'lift_off_mm': LaunchConfiguration('lift_off_mm'),
            }],
        ),
    ])
