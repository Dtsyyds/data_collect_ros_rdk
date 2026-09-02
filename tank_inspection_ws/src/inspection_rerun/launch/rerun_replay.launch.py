"""Launch the bridge and a bounded ros2 bag play process."""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _truth(value):
    return value.lower() in ("1", "true", "yes", "on")


def _launch_setup(context):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    if not os.path.isabs(bag_path):
        raise RuntimeError("bag_path must be an absolute path")
    if not os.path.exists(bag_path):
        raise RuntimeError("bag_path does not exist: %s" % bag_path)
    rate = float(LaunchConfiguration("playback_rate").perform(context))
    offset = float(LaunchConfiguration("start_offset").perform(context))
    duration = float(LaunchConfiguration("play_duration").perform(context))
    delay = float(LaunchConfiguration("playback_delay").perform(context))
    if rate <= 0.0 or offset < 0.0 or duration < 0.0 or delay < 0.0:
        raise RuntimeError("playback_rate must be positive; offsets/duration/delay cannot be negative")

    command = ["ros2", "bag", "play", bag_path, "--rate", str(rate),
               "--delay", str(delay), "--disable-keyboard-controls"]
    if _truth(LaunchConfiguration("loop").perform(context)):
        command.append("--loop")
    if offset > 0.0:
        command.extend(["--start-offset", str(offset)])
    if duration > 0.0:
        command.extend(["--playback-duration", str(duration)])
    use_clock = _truth(LaunchConfiguration("use_clock").perform(context))
    if use_clock:
        command.extend(["--clock", "100.0"])

    bridge = Node(
        package="inspection_rerun", executable="rerun_bridge", name="rerun_bridge",
        output="screen",
        parameters=[LaunchConfiguration("config_file"), {
            "output_mode": LaunchConfiguration("output_mode"),
            "save_path": LaunchConfiguration("save_path"),
            "use_sim_time": use_clock,
            "log_tank_scene": LaunchConfiguration("log_tank_scene"),
            "tank_asset_path": LaunchConfiguration("tank_asset_path"),
            "mission_path": LaunchConfiguration("mission_path"),
        }],
    )
    player = ExecuteProcess(cmd=command, output="screen")
    stop_when_bag_finishes = RegisterEventHandler(
        OnProcessExit(target_action=player,
                      on_exit=[EmitEvent(event=Shutdown(reason="bag playback finished"))]))
    return [bridge, player, stop_when_bag_finishes]


def generate_launch_description():
    default_config = PathJoinSubstitution([FindPackageShare("inspection_rerun"), "config", "rerun.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("bag_path"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("output_mode", default_value="spawn"),
        DeclareLaunchArgument("save_path", default_value="output/rerun/inspection.rrd"),
        DeclareLaunchArgument("log_tank_scene", default_value="false"),
        DeclareLaunchArgument("tank_asset_path", default_value=""),
        DeclareLaunchArgument("mission_path", default_value=""),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("playback_delay", default_value="2.0"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("start_offset", default_value="0.0"),
        DeclareLaunchArgument("play_duration", default_value="0.0"),
        DeclareLaunchArgument("use_clock", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
