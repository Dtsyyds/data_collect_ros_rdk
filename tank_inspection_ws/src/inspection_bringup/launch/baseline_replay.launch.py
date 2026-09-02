"""Replay the latest baseline-loop bag and reconstruct the D405 depth cloud."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_bag_root() -> Path:
    prefix = Path(get_package_prefix("inspection_bringup"))
    if prefix.parent.name == "install":
        return prefix.parent.parent / "bags"
    if prefix.name == "install":
        return prefix.parent / "bags"
    return Path.cwd() / "bags"


def _resolve_bag(value: str) -> Path:
    if value.strip():
        path = Path(value).expanduser().resolve()
        if not (path / "metadata.yaml").is_file():
            raise RuntimeError(f"not a closed rosbag2 directory: {path}")
        return path

    root = _default_bag_root()
    candidates = [
        path for path in root.glob("baseline_loop_*")
        if path.is_dir() and (path / "metadata.yaml").is_file()
    ]
    if not candidates:
        raise RuntimeError(
            "no closed baseline_loop_* bag found; first run "
            "'ros2 launch inspection_bringup baseline_capture.launch.py'")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _start_replay(context):
    bag = _resolve_bag(LaunchConfiguration("bag_directory").perform(context))
    return [
        LogInfo(msg=(
            f"BASELINE LOOP replay: {bag}; looping enabled. "
            "The green D405 XYZ cloud is reconstructed from recorded depth + CameraInfo."
        )),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", str(bag),
                "--loop", "--delay", "2.0", "--disable-keyboard-controls",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_directory", default_value="",
            description="Closed bag directory; empty selects the newest baseline_loop_* bag."),
        DeclareLaunchArgument(
            "point_stride", default_value="2",
            description="D405 replay point-cloud pixel stride (2 is suitable for USB2.1 data)."),
        DeclareLaunchArgument(
            "start_rviz", default_value="true",
            description="Start the baseline replay RViz view."),
        Node(
            package="inspection_tools",
            executable="depth_to_pointcloud",
            name="baseline_depth_to_pointcloud",
            output="screen",
            parameters=[{
                "stride": LaunchConfiguration("point_stride"),
                "output_topic": "/camera/reconstructed/depth/points",
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="baseline_replay_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            arguments=["-d", str(share / "config" / "baseline_replay.rviz")],
        ),
        OpaqueFunction(function=_start_replay),
    ])
