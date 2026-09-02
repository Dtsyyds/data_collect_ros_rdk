"""Launch the MID-360 + D405 depth/color recording profile and optional hardware.

MID-360, D405 depth/color and MCAP recording start by default. Eddy-current and
synchronization remain explicit opt-ins. Ultrasound and robot drivers remain
external adapters and must publish the documented topics.
"""

from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _default_bag_root() -> Path:
    """Locate this colcon workspace when possible, otherwise use the caller's directory."""
    prefix = Path(get_package_prefix("inspection_bringup"))
    if prefix.parent.name == "install":  # isolated install: <workspace>/install/<package>
        return prefix.parent.parent / "bags"
    if prefix.name == "install":  # merged install: <workspace>/install
        return prefix.parent / "bags"
    return Path.cwd() / "bags"


def _start_mid360(context):
    """Create the optional Livox node only after validating its explicit config."""
    if not _as_bool(LaunchConfiguration("start_mid360").perform(context)):
        return []

    config_value = LaunchConfiguration("mid360_config_path").perform(context).strip()
    if not config_value:
        raise RuntimeError(
            "start_mid360:=true requires mid360_config_path:=/absolute/path/MID360_config.json")
    config_path = Path(config_value).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"MID-360 config file does not exist: {config_path}")

    # Fail here, rather than at import/generation time, so the core launch remains
    # usable when this optional vendor package was not included in a partial build.
    get_package_share_directory("livox_ros_driver2")
    publish_freq = float(LaunchConfiguration("mid360_publish_freq").perform(context))
    if publish_freq <= 0.0:
        raise RuntimeError("mid360_publish_freq must be greater than zero")

    return [
        LogInfo(msg=(
            "Starting livox_ros_driver2 in PointCloud2 PointXYZRTLT mode; "
            f"config={config_path}"
        )),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "xfer_format": 0,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": publish_freq,
                "output_data_type": 0,
                "frame_id": LaunchConfiguration("mid360_frame_id"),
                "lvx_file_path": "",
                "user_config_path": str(config_path),
                "cmdline_input_bd_code": "",
            }],
            remappings=[
                ("/livox/lidar", LaunchConfiguration("lidar_topic")),
                ("/livox/imu", "/livox/imu_raw"),
            ],
        ),
        Node(
            package="inspection_livox_adapter",
            executable="livox_imu_adapter_node",
            name="livox_imu_adapter",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "acceleration_scale": LaunchConfiguration(
                    "mid360_imu_acceleration_scale"),
            }],
            remappings=[
                ("/livox/imu", LaunchConfiguration("imu_topic")),
            ],
        ),
    ]


def _publish_rough_extrinsics(context):
    """Publish a traceable rough MID-360-to-D405 transform when both devices run."""
    required_flags = ("start_mid360", "start_d405", "publish_rough_extrinsics")
    if not all(_as_bool(LaunchConfiguration(name).perform(context)) for name in required_flags):
        return []

    config_value = LaunchConfiguration("rough_extrinsics_path").perform(context).strip()
    if not config_value:
        raise RuntimeError("publish_rough_extrinsics:=true requires rough_extrinsics_path")
    config_path = Path(config_value).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"rough extrinsics file does not exist: {config_path}")

    document = yaml.safe_load(config_path.read_text()) or {}
    calibration = document.get("rough_extrinsics", {})
    if not calibration.get("enabled", False):
        return [LogInfo(msg=f"Rough extrinsics disabled in {config_path}")]
    if calibration.get("calibration_stage") != "rough":
        raise RuntimeError("rough extrinsics must declare calibration_stage: rough")
    if calibration.get("production_valid") is not False:
        raise RuntimeError("rough extrinsics must explicitly declare production_valid: false")

    transform = calibration.get("transform", {})
    expected_parent = str(transform.get("parent_frame", ""))
    parent_frame = LaunchConfiguration("mid360_frame_id").perform(context)
    if expected_parent != parent_frame:
        raise RuntimeError(
            f"rough extrinsics parent_frame {expected_parent!r} does not match "
            f"mid360_frame_id {parent_frame!r}")
    child_frame = str(transform.get("child_frame", ""))
    if not child_frame:
        raise RuntimeError("rough extrinsics child_frame must not be empty")

    translation = transform.get("translation_m", {})
    rotation = transform.get("rotation_rpy_rad", {})
    values = {
        "x": float(translation["x"]),
        "y": float(translation["y"]),
        "z": float(translation["z"]),
        "roll": float(rotation["roll"]),
        "pitch": float(rotation["pitch"]),
        "yaw": float(rotation["yaw"]),
    }
    calibration_id = str(calibration.get("calibration_id", "UNASSIGNED"))
    return [
        LogInfo(msg=(
            f"Publishing ROUGH extrinsics {calibration_id}: {parent_frame} -> {child_frame}; "
            f"xyz_m=({values['x']}, {values['y']}, {values['z']}), "
            f"rpy_rad=({values['roll']}, {values['pitch']}, {values['yaw']}); "
            "production_valid=false"
        )),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mid360_to_d405_rough_tf",
            output="screen",
            arguments=[
                "--x", str(values["x"]),
                "--y", str(values["y"]),
                "--z", str(values["z"]),
                "--roll", str(values["roll"]),
                "--pitch", str(values["pitch"]),
                "--yaw", str(values["yaw"]),
                "--frame-id", parent_frame,
                "--child-frame-id", child_frame,
            ],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    livox_share = Path(get_package_share_directory("livox_ros_driver2"))
    default_mid360_config = livox_share / "config" / "MID360_config.json"
    default_rough_extrinsics = share / "config" / "rough_extrinsics.yaml"
    default_output_directory = (
        _default_bag_root() /
        f"mid360_d405_run_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
    )
    record_launch = PythonLaunchDescriptionSource(
        str(share / "launch" / "record_only.launch.py"))

    start_d405 = LaunchConfiguration("start_d405")
    start_eddy = LaunchConfiguration("start_eddy")
    start_sync = LaunchConfiguration("start_sync")
    record = LaunchConfiguration("record")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_d405", default_value="true",
            description="Start the connected D405 depth and color streams (enabled by default)."),
        DeclareLaunchArgument(
            "d405_serial_no", default_value="",
            description="Optional RealSense serial selector; prefix a numeric serial with an underscore."),
        DeclareLaunchArgument(
            "d405_usb_port_id", default_value="",
            description="Optional RealSense physical USB port selector."),
        DeclareLaunchArgument(
            "d405_depth_profile", default_value="640x480x15",
            description="D405 depth profile in WIDTHxHEIGHTxFPS form."),
        DeclareLaunchArgument(
            "d405_color_profile", default_value="640x480x15",
            description="D405 color profile in WIDTHxHEIGHTxFPS form."),
        DeclareLaunchArgument(
            "d405_color_format", default_value="RGB8",
            description="D405 depth-module color format."),
        DeclareLaunchArgument(
            "d405_pointcloud_enable", default_value="false",
            description="Generate a derived D405 depth/color PointCloud2 for calibration viewing."),
        DeclareLaunchArgument("d405_diagnostics_period", default_value="1.0"),
        DeclareLaunchArgument(
            "start_mid360", default_value="true",
            description="Start livox_ros_driver2 in PointCloud2 mode (enabled by default)."),
        DeclareLaunchArgument(
            "mid360_config_path", default_value=str(default_mid360_config),
            description="Vendor JSON config path; defaults to the verified local MID-360 config."),
        DeclareLaunchArgument("mid360_publish_freq", default_value="10.0"),
        DeclareLaunchArgument("mid360_frame_id", default_value="livox_frame"),
        DeclareLaunchArgument(
            "publish_rough_extrinsics", default_value="true",
            description="Publish the traceable rough MID-360-to-D405 static transform."),
        DeclareLaunchArgument(
            "rough_extrinsics_path", default_value=str(default_rough_extrinsics),
            description="YAML containing the non-production rough extrinsic measurement."),
        DeclareLaunchArgument(
            "mid360_imu_acceleration_scale", default_value="9.80665",
            description="Convert the vendor IMU acceleration from g to m/s^2."),
        DeclareLaunchArgument(
            "start_eddy", default_value="false",
            description="Start the local STM32 USB-UART eddy-current driver."),
        DeclareLaunchArgument("eddy_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("eddy_baud", default_value="4000000"),
        DeclareLaunchArgument("eddy_publish_rate", default_value="10.0"),
        DeclareLaunchArgument("eddy_settings_query_rate", default_value="1.0"),
        DeclareLaunchArgument("eddy_sensor_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("eddy_calibration_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("eddy_frame_id", default_value="eddy_probe_link"),
        DeclareLaunchArgument("eddy_sampling_rate_hz", default_value="0.0"),
        DeclareLaunchArgument("eddy_gain_db", default_value="-1.0"),
        DeclareLaunchArgument("eddy_lift_off_mm", default_value="-1.0"),
        DeclareLaunchArgument("start_sync", default_value="false"),
        DeclareLaunchArgument(
            "anchor_decimation", default_value="1",
            description="Generate one FusionIndex for every N accepted anchor frames."),
        DeclareLaunchArgument(
            "software_clock_match_enable", default_value="false",
            description="Apply camera/Livox software time offsets against the anchor clock."),
        DeclareLaunchArgument(
            "software_clock_adaptive_enable", default_value="false",
            description="Continuously adapt offsets; experimental and disabled by default."),
        DeclareLaunchArgument("camera_clock_offset_ms", default_value="0.0"),
        DeclareLaunchArgument("livox_clock_offset_ms", default_value="0.0"),
        DeclareLaunchArgument("clock_offset_alpha", default_value="0.25"),
        DeclareLaunchArgument("clock_offset_max_ms", default_value="20.0"),
        DeclareLaunchArgument("clock_offset_update_gate_ms", default_value="45.0"),
        DeclareLaunchArgument("clock_offset_window_matches", default_value="50"),
        DeclareLaunchArgument(
            "anchor_topic", default_value="/inspection/eddy_current/raw",
            description="Fusion anchor: /inspection/eddy_current/raw or /inspection/ultrasound/raw."),
        DeclareLaunchArgument(
            "record", default_value="true",
            description="Record configured topics to split MCAP files (enabled by default)."),
        DeclareLaunchArgument("debug_mode", default_value="false"),
        DeclareLaunchArgument(
            "output_directory", default_value=str(default_output_directory),
            description="MCAP output directory; defaults to a new timestamped directory."),
        DeclareLaunchArgument("asset_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("course_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("plate_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("weld_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument(
            "camera_image_topic", default_value="/camera/camera/depth/image_rect_raw"),
        DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
        DeclareLaunchArgument("odometry_topic", default_value="/robot/odometry"),
        DeclareLaunchArgument("robot_state_topic", default_value="/robot/state"),
        DeclareLaunchArgument(
            "ultrasound_topic", default_value="/inspection/ultrasound/raw"),
        DeclareLaunchArgument(
            "eddy_current_topic", default_value="/inspection/eddy_current/raw"),
        DeclareLaunchArgument(
            "probe_state_topic", default_value="/inspection/probe_state"),
        LogInfo(msg=[
            "Default profile: MID-360 + D405 depth/color + IMU SI adapter + MCAP recording; output=",
            LaunchConfiguration("output_directory"),
            ". Eddy-current and synchronization remain opt-in.",
        ]),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="camera",
            name="camera",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(start_d405),
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[{
                "serial_no": LaunchConfiguration("d405_serial_no"),
                "usb_port_id": LaunchConfiguration("d405_usb_port_id"),
                "enable_depth": True,
                "enable_color": True,
                "enable_infra": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "depth_module.depth_profile": LaunchConfiguration("d405_depth_profile"),
                "depth_module.depth_format": "Z16",
                # D405 exposes color through its stereo/depth module, rather than rgb_camera.
                "depth_module.color_profile": LaunchConfiguration("d405_color_profile"),
                "depth_module.color_format": LaunchConfiguration("d405_color_format"),
                "diagnostics_period": LaunchConfiguration("d405_diagnostics_period"),
                "pointcloud.enable": LaunchConfiguration("d405_pointcloud_enable"),
                "pointcloud.stream_filter": 2,
                "publish_tf": True,
            }],
        ),
        OpaqueFunction(function=_start_mid360),
        # This identity transform gives the otherwise standalone Livox frame a TF-tree
        # anchor for RViz. It is not a measured D405-to-MID360 extrinsic calibration.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mid360_visualization_tf",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_mid360")),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "inspection_origin",
                "--child-frame-id", LaunchConfiguration("mid360_frame_id"),
            ],
        ),
        OpaqueFunction(function=_publish_rough_extrinsics),
        Node(
            package="eddy_driver",
            executable="eddy_driver_node",
            name="eddy_driver_node",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(start_eddy),
            parameters=[{
                "device": LaunchConfiguration("eddy_device"),
                "baud": LaunchConfiguration("eddy_baud"),
                "publish_rate": LaunchConfiguration("eddy_publish_rate"),
                "settings_query_rate": LaunchConfiguration("eddy_settings_query_rate"),
                "sensor_id": LaunchConfiguration("eddy_sensor_id"),
                "calibration_id": LaunchConfiguration("eddy_calibration_id"),
                "frame_id": LaunchConfiguration("eddy_frame_id"),
                "sampling_rate_hz": LaunchConfiguration("eddy_sampling_rate_hz"),
                "gain_db": LaunchConfiguration("eddy_gain_db"),
                "lift_off_mm": LaunchConfiguration("eddy_lift_off_mm"),
            }],
            remappings=[
                ("/inspection/eddy_current/raw", LaunchConfiguration("eddy_current_topic")),
            ],
        ),
        Node(
            package="inspection_sync",
            executable="inspection_sync_node",
            name="inspection_sync",
            output="screen",
            condition=IfCondition(start_sync),
            parameters=[
                str(share / "config" / "cache.yaml"),
                {
                    "anchor_topic": LaunchConfiguration("anchor_topic"),
                    "anchor_decimation": LaunchConfiguration("anchor_decimation"),
                    "software_clock_match_enable": LaunchConfiguration(
                        "software_clock_match_enable"),
                    "software_clock_adaptive_enable": LaunchConfiguration(
                        "software_clock_adaptive_enable"),
                    "camera_clock_offset_ms": LaunchConfiguration("camera_clock_offset_ms"),
                    "livox_clock_offset_ms": LaunchConfiguration("livox_clock_offset_ms"),
                    "clock_offset_alpha": LaunchConfiguration("clock_offset_alpha"),
                    "clock_offset_max_ms": LaunchConfiguration("clock_offset_max_ms"),
                    "clock_offset_update_gate_ms": LaunchConfiguration(
                        "clock_offset_update_gate_ms"),
                    "clock_offset_window_matches": LaunchConfiguration(
                        "clock_offset_window_matches"),
                    "asset_id": LaunchConfiguration("asset_id"),
                    "course_id": LaunchConfiguration("course_id"),
                    "plate_id": LaunchConfiguration("plate_id"),
                    "weld_id": LaunchConfiguration("weld_id"),
                    "asset_coordinate_valid": False,
                    "camera_reliable_qos": True,
                },
            ],
            remappings=[
                ("/camera/camera/depth/image_rect_raw", LaunchConfiguration("camera_image_topic")),
                ("/livox/lidar", LaunchConfiguration("lidar_topic")),
                ("/livox/imu", LaunchConfiguration("imu_topic")),
                ("/robot/odometry", LaunchConfiguration("odometry_topic")),
                ("/robot/state", LaunchConfiguration("robot_state_topic")),
                ("/inspection/ultrasound/raw", LaunchConfiguration("ultrasound_topic")),
                ("/inspection/eddy_current/raw", LaunchConfiguration("eddy_current_topic")),
                ("/inspection/probe_state", LaunchConfiguration("probe_state_topic")),
            ],
        ),
        IncludeLaunchDescription(
            record_launch,
            condition=IfCondition(record),
            launch_arguments={
                "debug_mode": LaunchConfiguration("debug_mode"),
                "output_directory": LaunchConfiguration("output_directory"),
            }.items(),
        ),
    ])
