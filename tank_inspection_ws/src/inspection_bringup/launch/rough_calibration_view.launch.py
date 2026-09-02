"""Start the real sensors and RViz for non-production rough extrinsic inspection."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    return LaunchDescription([
        LogInfo(msg=(
            "ROUGH calibration view: recording and synchronization are disabled; "
            "the displayed transform is production_valid=false."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch" / "hardware_pipeline.launch.py")),
            launch_arguments={
                "record": "false",
                "start_sync": "false",
                "publish_rough_extrinsics": "true",
                "d405_pointcloud_enable": "true",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rough_calibration_rviz",
            output="screen",
            arguments=["-d", str(share / "config" / "rough_calibration.rviz")],
        ),
    ])
