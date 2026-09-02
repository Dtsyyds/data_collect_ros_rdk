"""Capture one time-bounded MID-360 + D405 baseline-loop bag."""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_bag_root() -> Path:
    prefix = Path(get_package_prefix("inspection_bringup"))
    if prefix.parent.name == "install":
        return prefix.parent.parent / "bags"
    if prefix.name == "install":
        return prefix.parent / "bags"
    return Path.cwd() / "bags"


def _auto_stop(context):
    seconds = float(LaunchConfiguration("capture_seconds").perform(context))
    if seconds < 0.0:
        raise RuntimeError("capture_seconds must be non-negative")
    if seconds == 0.0:
        return [LogInfo(msg="Baseline capture will run until Ctrl-C (capture_seconds=0).")]
    return [
        LogInfo(msg=f"Baseline capture will stop cleanly after {seconds:g} seconds."),
        TimerAction(
            period=seconds,
            actions=[EmitEvent(event=Shutdown(
                reason=f"baseline capture duration {seconds:g}s completed"))],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    output = _default_bag_root().joinpath(
        f"baseline_loop_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}")
    return LaunchDescription([
        DeclareLaunchArgument(
            "capture_seconds", default_value="60",
            description="Automatic clean shutdown delay; zero means run until Ctrl-C."),
        DeclareLaunchArgument("output_directory", default_value=str(output)),
        DeclareLaunchArgument("start_mid360", default_value="true"),
        DeclareLaunchArgument("start_d405", default_value="true"),
        LogInfo(msg=[
            "BASELINE LOOP capture: raw depth/color, camera info, MID-360, IMU and TF; output=",
            LaunchConfiguration("output_directory"),
            ". Reconstructed D405 point clouds are intentionally not recorded.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch" / "hardware_pipeline.launch.py")),
            launch_arguments={
                "start_mid360": LaunchConfiguration("start_mid360"),
                "start_d405": LaunchConfiguration("start_d405"),
                "start_eddy": "false",
                "start_sync": "false",
                "record": "true",
                "debug_mode": "false",
                "d405_pointcloud_enable": "false",
                "publish_rough_extrinsics": "true",
                "output_directory": LaunchConfiguration("output_directory"),
            }.items(),
        ),
        OpaqueFunction(function=_auto_stop),
    ])
