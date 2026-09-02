"""Capture a time-bounded provisional Eddy-anchored synchronization bag."""

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
        return [LogInfo(msg="Eddy synchronization capture will run until Ctrl-C.")]
    return [
        LogInfo(msg=f"Eddy synchronization capture will stop after {seconds:g} seconds."),
        TimerAction(
            period=seconds,
            actions=[EmitEvent(event=Shutdown(
                reason=f"Eddy synchronization capture duration {seconds:g}s completed"))],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    output = _default_bag_root().joinpath(
        f"eddy_sync_loop_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}")
    return LaunchDescription([
        DeclareLaunchArgument(
            "capture_seconds", default_value="60",
            description="Automatic clean shutdown delay; zero means run until Ctrl-C."),
        DeclareLaunchArgument("output_directory", default_value=str(output)),
        DeclareLaunchArgument("eddy_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument(
            "camera_clock_offset_ms", default_value="0.0",
            description="Optional fixed D405 stamp correction used only by FusionIndex."),
        DeclareLaunchArgument(
            "livox_clock_offset_ms", default_value="0.0",
            description="Optional fixed MID-360/IMU stamp correction used only by FusionIndex."),
        LogInfo(msg=[
            "PROVISIONAL EDDY TIME LOOP: Eddy host-receive timestamps anchor D405, MID-360 ",
            "and IMU bounded nearest-stamp matching; output=",
            LaunchConfiguration("output_directory"),
            ". Pose remains invalid without /robot/odometry. Eddy calibration remains unassigned.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch" / "hardware_pipeline.launch.py")),
            launch_arguments={
                "start_mid360": "true",
                "start_d405": "true",
                "start_eddy": "true",
                "eddy_device": LaunchConfiguration("eddy_device"),
                "eddy_sensor_id": "UNASSIGNED",
                "eddy_calibration_id": "UNASSIGNED",
                "start_sync": "true",
                "anchor_topic": "/inspection/eddy_current/raw",
                "anchor_decimation": "10",
                "software_clock_match_enable": "true",
                "software_clock_adaptive_enable": "false",
                # Periodic streams make a constant offset phase-ambiguous. Keep the validated
                # nearest-stamp matcher phase-safe by default; raw header stamps are never changed.
                "camera_clock_offset_ms": LaunchConfiguration("camera_clock_offset_ms"),
                "livox_clock_offset_ms": LaunchConfiguration("livox_clock_offset_ms"),
                "record": "true",
                "debug_mode": "false",
                "d405_pointcloud_enable": "false",
                "publish_rough_extrinsics": "true",
                "output_directory": LaunchConfiguration("output_directory"),
            }.items(),
        ),
        OpaqueFunction(function=_auto_stop),
    ])
