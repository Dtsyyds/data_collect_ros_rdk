"""Capture PAUT UDP ultrasound frames to split MCAP files (standalone, no camera/lidar)."""

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
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        return [LogInfo(msg="PAUT capture will run until Ctrl-C (capture_seconds=0).")]
    return [
        LogInfo(msg=f"PAUT capture will stop cleanly after {seconds:g} seconds."),
        TimerAction(
            period=seconds,
            actions=[EmitEvent(event=Shutdown(
                reason=f"paut capture duration {seconds:g}s completed"))],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    output = _default_bag_root().joinpath(
        f"paut_run_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}")
    record_launch = PythonLaunchDescriptionSource(
        str(share / "launch" / "record_only.launch.py"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "capture_seconds", default_value="0",
            description="Automatic clean shutdown delay; zero means run until Ctrl-C."),
        DeclareLaunchArgument("output_directory", default_value=str(output)),
        DeclareLaunchArgument("debug_mode", default_value="false"),
        DeclareLaunchArgument("record", default_value="true"),
        # 设备端 v1 通道。老的 :12345(v0 裁剪图像) 已不再采集。
        DeclareLaunchArgument("port", default_value="12346"),
        DeclareLaunchArgument("sensor_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("calibration_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("frame_id", default_value="paut_probe_link"),
        # 注: v0 的 sample_encoding / sampling_rate_hz / gain_db / sound_velocity_m_s
        #     已移除 -- 这些量现在由设备的 CONFIG 包上报（/inspection/paut/config）。
        LogInfo(msg=[
            "PAUT capture: UDP :12346 -> /inspection/paut/raw_v2 + /inspection/paut/config"
            " -> split MCAP; output=",
            LaunchConfiguration("output_directory"),
        ]),
        Node(
            package="paut_driver",
            executable="paut_driver_node",
            name="paut_driver_node",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "sensor_id": LaunchConfiguration("sensor_id"),
                "calibration_id": LaunchConfiguration("calibration_id"),
                "frame_id": LaunchConfiguration("frame_id"),
            }],
        ),
        IncludeLaunchDescription(
            record_launch,
            condition=IfCondition(LaunchConfiguration("record")),
            launch_arguments={
                "debug_mode": LaunchConfiguration("debug_mode"),
                "output_directory": LaunchConfiguration("output_directory"),
            }.items(),
        ),
        OpaqueFunction(function=_auto_stop),
    ])
