#!/usr/bin/env python3
"""Launch the PAUT UDP v1 receive driver in isolation (no recorder).

发布 /inspection/paut/raw_v2 (完整 61x896 原始数据) 与 /inspection/paut/config。

注: 采集端标签(sensor_id/calibration_id/frame_id)的 launch 默认值会**覆盖**
    config/paut_params.yaml 里的同名项。要改配置请改这里的默认值，或从命令行传参。
"""
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
        # 设备端 v1 通道。老的 :12345(v0 裁剪图像) 已不再使用。
        DeclareLaunchArgument('port', default_value='12346'),
        DeclareLaunchArgument('sensor_id', default_value='UNASSIGNED'),
        DeclareLaunchArgument('calibration_id', default_value='UNASSIGNED'),
        DeclareLaunchArgument('frame_id', default_value='paut_probe_link'),
        # 注: v0 的 sample_encoding / sampling_rate_hz / gain_db / sound_velocity_m_s
        #     已移除 -- 这些量现在由设备的 CONFIG 包上报（PautConfig 话题）。
        Node(
            package='paut_driver', executable='paut_driver_node',
            name='paut_driver_node', output='screen',
            parameters=[params, {
                'port': LaunchConfiguration('port'),
                'sensor_id': LaunchConfiguration('sensor_id'),
                'calibration_id': LaunchConfiguration('calibration_id'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
