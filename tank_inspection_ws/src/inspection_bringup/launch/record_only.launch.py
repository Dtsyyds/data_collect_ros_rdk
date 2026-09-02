"""Record the configured raw and derived topics to split MCAP files."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _record_process(context):
    share = Path(get_package_share_directory("inspection_bringup"))
    settings = yaml.safe_load((share / "config" / "record_topics.yaml").read_text())
    profile = "debug" if _as_bool(LaunchConfiguration("debug_mode").perform(context)) else "production"
    split = settings[profile]
    return [ExecuteProcess(cmd=[
        "ros2", "bag", "record", "--storage", "mcap",
        "--output", LaunchConfiguration("output_directory"),
        "--max-bag-size", str(split["max_bag_size"]),
        "--max-bag-duration", str(split["max_bag_duration"]),
        "--storage-config-file", str(share / "config" / "mcap_writer_options.yaml"),
        "--qos-profile-overrides-path", str(share / "config" / "qos_overrides.yaml"),
        "--topics", *settings["topics"],
    ], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_directory", default_value="bags/record_only"),
        DeclareLaunchArgument("debug_mode", default_value="false"),
        OpaqueFunction(function=_record_process),
    ])
