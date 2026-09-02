"""Launch only the Rerun bridge; no drivers and no rosbag recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([FindPackageShare("inspection_rerun"), "config", "rerun.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("output_mode", default_value="spawn"),
        DeclareLaunchArgument("save_path", default_value="output/rerun/inspection.rrd"),
        DeclareLaunchArgument("log_tank_scene", default_value="false"),
        DeclareLaunchArgument("tank_asset_path", default_value=""),
        DeclareLaunchArgument("mission_path", default_value=""),
        Node(
            package="inspection_rerun",
            executable="rerun_bridge",
            name="rerun_bridge",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), {
                "output_mode": LaunchConfiguration("output_mode"),
                "save_path": LaunchConfiguration("save_path"),
                "log_tank_scene": LaunchConfiguration("log_tank_scene"),
                "tank_asset_path": LaunchConfiguration("tank_asset_path"),
                "mission_path": LaunchConfiguration("mission_path"),
            }],
        ),
    ])
