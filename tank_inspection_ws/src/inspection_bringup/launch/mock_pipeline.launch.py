"""Launch deterministic mock sensors, bounded synchronization, and optional MCAP recording."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _record_process(context):
    if not _as_bool(LaunchConfiguration("record").perform(context)):
        return []
    share = Path(get_package_share_directory("inspection_bringup"))
    settings = yaml.safe_load((share / "config" / "record_topics.yaml").read_text())
    profile = "debug" if _as_bool(LaunchConfiguration("debug_mode").perform(context)) else "production"
    split = settings[profile]
    command = [
        "ros2", "bag", "record", "--storage", "mcap",
        "--output", LaunchConfiguration("output_directory"),
        "--max-bag-size", str(split["max_bag_size"]),
        "--max-bag-duration", str(split["max_bag_duration"]),
        "--storage-config-file", str(share / "config" / "mcap_writer_options.yaml"),
        "--qos-profile-overrides-path", str(share / "config" / "qos_overrides.yaml"),
        "--topics", *settings["topics"],
    ]
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("output_directory", default_value="bags/mock_run"),
        DeclareLaunchArgument("debug_mode", default_value="true"),
        DeclareLaunchArgument("mission_trajectory_path", default_value=""),
        DeclareLaunchArgument("mission_speed_m_s", default_value="2.0"),
        DeclareLaunchArgument("mission_dwell_s", default_value="0.5"),
        DeclareLaunchArgument("mission_loop", default_value="true"),
        Node(
            package="inspection_mock_sensors",
            executable="mock_sensors_node",
            name="inspection_mock_sensors",
            output="screen",
            parameters=[
                str(share / "config" / "mock_sensors.yaml"),
                {
                    "mission_trajectory_path": LaunchConfiguration("mission_trajectory_path"),
                    "mission_speed_m_s": LaunchConfiguration("mission_speed_m_s"),
                    "mission_dwell_s": LaunchConfiguration("mission_dwell_s"),
                    "mission_loop": LaunchConfiguration("mission_loop"),
                },
            ],
        ),
        Node(
            package="inspection_sync",
            executable="inspection_sync_node",
            name="inspection_sync",
            output="screen",
            parameters=[str(share / "config" / "cache.yaml")],
        ),
        OpaqueFunction(function=_record_process),
    ])
