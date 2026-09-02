"""ROS 2 bridge node that mirrors selected topics into an optional Rerun sink."""

import math
import time
from collections import Counter
from typing import Any, Callable, Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

from .camera_adapter import camera_info_entity
from .config_validation import validate_parameters
from .diagnostic_adapter import diagnostic_level
from .image_adapter import image_to_numpy, is_depth_encoding
from .imu_adapter import log_imu
from .inspection_adapter import (log_eddy, log_fusion_index, log_probe,
                                 log_robot, log_ultrasound)
from .navigation_adapter import log_path, log_pose
from .pointcloud_adapter import extract_pointcloud
from .rerun_session import (RerunSession, coordinate_frame_entity, image_entity,
                            points3d_entity, scalar_entity, text_entity)
from .tank_scene import load_tank_scene
from .tank_scene_adapter import (ActualTrajectoryLogger, build_tank_blueprint,
                                 log_asset_context, log_mission)
from .tf_adapter import log_tf_message
from .timestamp_utils import RateLimiter, TimelineState, message_time_ns, stamp_to_ns

try:
    from inspection_interfaces.msg import (EddyCurrentFrame, FusionIndex, ProbeState,
                                           RobotState, UltrasoundFrame)
except ImportError as exc:
    _CUSTOM_IMPORT_ERROR = exc
    EddyCurrentFrame = FusionIndex = ProbeState = RobotState = UltrasoundFrame = None
else:
    _CUSTOM_IMPORT_ERROR = None


class RerunBridge(Node):
    def __init__(self) -> None:
        super().__init__("rerun_bridge")
        defaults = {
            "output_mode": "spawn", "recording_id": "tank_inspection_local",
            "save_path": "output/rerun/inspection.rrd",
            "connect_url": "rerun+http://127.0.0.1:9876/proxy",
            "image_topic": "/camera/camera/depth/image_rect_raw",
            "camera_info_topic": "/camera/camera/depth/camera_info",
            "pointcloud_topic": "/livox/lidar", "imu_topic": "/livox/imu",
            "imu_raw_topic": "/livox/imu_raw", "tf_topic": "/tf",
            "tf_static_topic": "/tf_static", "eddy_topic": "/inspection/eddy_current/raw",
            "ultrasound_topic": "/inspection/ultrasound/raw",
            "probe_state_topic": "/inspection/probe_state",
            "robot_state_topic": "/robot/state",
            "fusion_index_topic": "/derived/inspection/fusion_index",
            "diagnostics_topic": "/diagnostics", "odometry_topic": "/robot/odometry",
            "path_topic": "/path", "pose_topic": "/pose",
            "image_max_hz": 2.0, "pointcloud_max_hz": 1.0, "imu_max_hz": 20.0,
            "depth_units_per_meter": 1000.0,
            "max_point_count": 100000, "point_stride": 1, "point_voxel_size": 0.0,
            "log_image": True, "log_camera_info": True, "log_pointcloud": True,
            "log_imu": True, "log_tf": True, "log_eddy": True,
            "log_ultrasound": True, "log_probe_state": True, "log_robot_state": True,
            "log_fusion_index": True, "log_diagnostics": True,
            "log_navigation": True,
            "log_tank_scene": False, "tank_asset_path": "", "mission_path": "",
            "fusion_height_field": "v_m", "actual_trajectory_min_step_m": 0.01,
            "actual_trajectory_max_segments": 20000,
            "actual_trajectory_max_step_m": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        validate_parameters(self.params)
        custom_enabled = any(self.params[name] for name in (
            "log_eddy", "log_ultrasound", "log_probe_state", "log_robot_state",
            "log_fusion_index"))
        if custom_enabled and _CUSTOM_IMPORT_ERROR is not None:
            raise RuntimeError(
                "inspection_interfaces Python type support is unavailable; build and source "
                "inspection_interfaces before inspection_rerun: %s" % _CUSTOM_IMPORT_ERROR)

        self.tank_scene = None
        self.actual_trajectory = None
        blueprint = None
        if self.params["log_tank_scene"]:
            self.tank_scene = load_tank_scene(
                str(self.params["tank_asset_path"]), str(self.params["mission_path"]))
            blueprint = build_tank_blueprint(self.tank_scene)
            self.actual_trajectory = ActualTrajectoryLogger(
                self.tank_scene,
                float(self.params["actual_trajectory_min_step_m"]),
                int(self.params["actual_trajectory_max_segments"]),
                str(self.params["fusion_height_field"]),
                float(self.params["actual_trajectory_max_step_m"]))

        self.session = RerunSession(
            str(self.params["output_mode"]), str(self.params["recording_id"]),
            str(self.params["save_path"]), str(self.params["connect_url"]), blueprint)
        if self.tank_scene is not None:
            log_asset_context(self.session, self.tank_scene)
            log_mission(self.session, self.tank_scene)
        self.timeline = TimelineState()
        self.stats = Counter()
        self.last_message_ns = {}  # type: Dict[str, int]
        self._warning_times = {}  # type: Dict[str, float]
        self._camera_zero_warned = False
        self.limiters = {
            "image": RateLimiter(float(self.params["image_max_hz"])),
            "pointcloud": RateLimiter(float(self.params["pointcloud_max_hz"])),
            "imu": RateLimiter(float(self.params["imu_max_hz"])),
            "imu_raw": RateLimiter(float(self.params["imu_max_hz"])),
        }
        self._bridge_subscriptions = []
        self._create_subscriptions()
        self.get_logger().info("Rerun bridge started in %s mode%s" % (
            self.params["output_mode"],
            " with tank %s" % self.tank_scene.tank_id if self.tank_scene else ""))

    def _subscribe(self, msg_type: Any, topic_key: str, callback: Callable,
                   qos: QoSProfile) -> None:
        topic = str(self.params[topic_key])
        if not topic:
            raise ValueError("%s cannot be empty" % topic_key)
        self._bridge_subscriptions.append(self.create_subscription(msg_type, topic, callback, qos))

    def _create_subscriptions(self) -> None:
        sensor_qos = qos_profile_sensor_data
        tf_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable_small = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                    reliability=ReliabilityPolicy.RELIABLE)
        if self.params["log_image"]:
            self._subscribe(Image, "image_topic", self._image_callback, sensor_qos)
        if self.params["log_camera_info"]:
            self._subscribe(CameraInfo, "camera_info_topic", self._camera_callback, sensor_qos)
        if self.params["log_pointcloud"]:
            self._subscribe(PointCloud2, "pointcloud_topic", self._pointcloud_callback, sensor_qos)
        if self.params["log_imu"]:
            self._subscribe(Imu, "imu_topic", lambda msg: self._imu_callback(msg, "imu"), sensor_qos)
            self._subscribe(Imu, "imu_raw_topic", lambda msg: self._imu_callback(msg, "imu_raw"), sensor_qos)
        if self.params["log_tf"]:
            self._subscribe(TFMessage, "tf_topic", lambda msg: self._tf_callback(msg, False), tf_qos)
            self._subscribe(TFMessage, "tf_static_topic", lambda msg: self._tf_callback(msg, True), static_qos)
        if self.params["log_diagnostics"]:
            self._subscribe(DiagnosticArray, "diagnostics_topic", self._diagnostics_callback, reliable_small)
        if self.params["log_eddy"]:
            self._subscribe(EddyCurrentFrame, "eddy_topic", self._eddy_callback, sensor_qos)
        if self.params["log_ultrasound"]:
            self._subscribe(UltrasoundFrame, "ultrasound_topic", self._ultrasound_callback, sensor_qos)
        if self.params["log_probe_state"]:
            self._subscribe(ProbeState, "probe_state_topic", self._probe_callback, sensor_qos)
        if self.params["log_robot_state"]:
            self._subscribe(RobotState, "robot_state_topic", self._robot_callback, sensor_qos)
        if self.params["log_fusion_index"]:
            self._subscribe(FusionIndex, "fusion_index_topic", self._fusion_callback, reliable_small)
        if self.params["log_navigation"]:
            self._subscribe(Odometry, "odometry_topic", self._odometry_callback, sensor_qos)
            self._subscribe(Path, "path_topic", self._path_callback, reliable_small)
            self._subscribe(PoseStamped, "pose_topic", self._pose_callback, sensor_qos)

    def _receive_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _message_ns(self, message: Any, kind: str, explicit_stamp: Any = None) -> int:
        receive_ns = self._receive_ns()
        if explicit_stamp is not None:
            value = stamp_to_ns(explicit_stamp)
            used_receive = value == 0
            time_ns = receive_ns if used_receive else value
        else:
            time_ns, used_receive = message_time_ns(message, receive_ns)
        if used_receive:
            self._warn_throttled("fallback_" + kind,
                                 "%s has no non-zero header stamp; using receive time" % kind)
            self.stats[kind + ".receive_time_fallback"] += 1
        self.last_message_ns[kind] = time_ns
        return time_ns

    def _set_time(self, time_ns: int) -> None:
        sequence, rollback = self.timeline.advance(time_ns)
        if rollback:
            self.stats["timeline.rollbacks"] += 1
            self._warn_throttled("rollback", "ROS time moved backwards; continuing on ros_time timeline")
        self.session.set_time(time_ns, sequence)

    def _warn_throttled(self, key: str, message: str, period: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._warning_times.get(key, -1e30) >= period:
            self.get_logger().warning(message)
            self._warning_times[key] = now

    def _guard(self, kind: str, action: Callable[[], Any]) -> None:
        self.stats[kind + ".received"] += 1
        try:
            result = action()
            if result is not False:
                self.stats[kind + ".logged"] += 1
        except (ValueError, RuntimeError, TypeError, OverflowError) as exc:
            self.stats[kind + ".parse_failures"] += 1
            self.get_logger().error("%s message skipped: %s" % (kind, exc))

    def _image_callback(self, message: Image) -> None:
        def action() -> None:
            time_ns = self._message_ns(message, "image")
            if not self.limiters["image"].allow(time_ns):
                self.stats["image.skipped_rate"] += 1
                return False
            self._set_time(time_ns)
            depth_scale = (float(self.params["depth_units_per_meter"])
                           if is_depth_encoding(message.encoding) else None)
            self.session.log("world/camera/image",
                             image_entity(image_to_numpy(message), depth_scale))
        self._guard("image", action)

    def _camera_callback(self, message: CameraInfo) -> None:
        def action() -> None:
            self._set_time(self._message_ns(message, "camera_info"))
            entity = camera_info_entity(message)
            if entity is None:
                if not self._camera_zero_warned:
                    self.get_logger().warning("CameraInfo intrinsics are zero/invalid; pinhole not logged")
                    self._camera_zero_warned = True
                self.stats["camera_info.invalid_intrinsics"] += 1
                return False
            self.session.log("world/camera", entity)
        self._guard("camera_info", action)

    def _pointcloud_callback(self, message: PointCloud2) -> None:
        def action() -> None:
            time_ns = self._message_ns(message, "pointcloud")
            if not self.limiters["pointcloud"].allow(time_ns):
                self.stats["pointcloud.skipped_rate"] += 1
                return False
            self._set_time(time_ns)
            cloud = extract_pointcloud(
                message, int(self.params["point_stride"]), int(self.params["max_point_count"]),
                float(self.params["point_voxel_size"]))
            self.session.log(
                "world/lidar", points3d_entity(cloud.positions, cloud.colors),
                coordinate_frame_entity(str(message.header.frame_id)))
            self.stats["pointcloud.points_logged"] += len(cloud.positions)
        self._guard("pointcloud", action)

    def _imu_callback(self, message: Imu, kind: str) -> None:
        def action() -> None:
            time_ns = self._message_ns(message, kind)
            if not self.limiters[kind].allow(time_ns):
                self.stats[kind + ".skipped_rate"] += 1
                return False
            self._set_time(time_ns)
            log_imu(self.session, message, "signals/%s" % kind)
        self._guard(kind, action)

    def _tf_callback(self, message: TFMessage, is_static: bool) -> None:
        kind = "tf_static" if is_static else "tf"
        def action() -> None:
            stamp = message.transforms[0].header.stamp if message.transforms else None
            self._set_time(self._message_ns(message, kind, stamp))
            count = log_tf_message(self.session, message, is_static,
                                   lambda text: self._warn_throttled("tf_invalid", text))
            self.stats[kind + ".transforms"] += count
        self._guard(kind, action)

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        def action() -> None:
            self._set_time(self._message_ns(message, "diagnostics"))
            for status in message.status:
                numeric_level = diagnostic_level(status.level)
                level = "ERROR" if numeric_level >= 2 else ("WARN" if numeric_level == 1 else "INFO")
                details = ", ".join("%s=%s" % (item.key, item.value) for item in status.values)
                text = "%s: %s%s" % (status.name, status.message, " (" + details + ")" if details else "")
                self.session.log("system/diagnostics", text_entity(text, level))
        self._guard("diagnostics", action)

    def _custom(self, message: Any, kind: str, adapter: Callable[[Any, Any], None]) -> None:
        def action() -> None:
            self._set_time(self._message_ns(message, kind))
            adapter(self.session, message)
        self._guard(kind, action)

    def _eddy_callback(self, message: Any) -> None:
        def action() -> None:
            receive_ns = self._receive_ns()
            header_ns, used_receive = message_time_ns(message, receive_ns)
            if used_receive:
                self._warn_throttled("fallback_eddy",
                                     "eddy has no non-zero header stamp; using receive time")
                self.stats["eddy.receive_time_fallback"] += 1
            self.last_message_ns["eddy"] = header_ns
            self._set_time(header_ns)
            log_eddy(self.session, message)
            self.session.log("signals/eddy/timestamps/ros_header_ns", text_entity(str(header_ns)))
            self.session.log("signals/eddy/timestamps/ros_receive_ns", text_entity(str(receive_ns)))
            self.session.log("signals/eddy/timestamps/receive_minus_header_ns",
                             scalar_entity(receive_ns - header_ns))
        self._guard("eddy", action)

    def _ultrasound_callback(self, message: Any) -> None:
        self._custom(message, "ultrasound", log_ultrasound)

    def _probe_callback(self, message: Any) -> None:
        self._custom(message, "probe", log_probe)

    def _robot_callback(self, message: Any) -> None:
        self._custom(message, "robot", log_robot)

    def _fusion_callback(self, message: Any) -> None:
        def action() -> None:
            self._set_time(self._message_ns(message, "fusion_index"))
            log_fusion_index(self.session, message)
            if self.actual_trajectory is not None:
                try:
                    self.actual_trajectory.log(self.session, message)
                    self.stats["actual_trajectory.logged"] += 1
                except ValueError as exc:
                    self.stats["actual_trajectory.skipped_invalid"] += 1
                    self._warn_throttled("actual_trajectory_invalid",
                                         "FusionIndex not added to tank trajectory: %s" % exc)
        self._guard("fusion_index", action)

    def _odometry_callback(self, message: Odometry) -> None:
        def action() -> None:
            self._set_time(self._message_ns(message, "odometry"))
            # /tf is authoritative when enabled. Logging the same odometry pose
            # as another named transform gives base_link two temporal owners.
            if not self.params["log_tf"]:
                log_pose(self.session, message.pose.pose, message.header.frame_id,
                         message.child_frame_id or "robot", "world/robot")
            else:
                self.stats["odometry.transform_skipped_tf_authoritative"] += 1
        self._guard("odometry", action)

    def _path_callback(self, message: Path) -> None:
        def action() -> None:
            self._set_time(self._message_ns(message, "path"))
            self.stats["path.poses"] += log_path(self.session, message.poses)
        self._guard("path", action)

    def _pose_callback(self, message: PoseStamped) -> None:
        def action() -> None:
            self._set_time(self._message_ns(message, "pose"))
            log_pose(self.session, message.pose, message.header.frame_id,
                     "robot_pose", "world/robot/pose")
        self._guard("pose", action)

    def destroy_node(self) -> bool:
        parts = ["%s=%s" % item for item in sorted(self.stats.items())]
        latest = ["%s=%d" % item for item in sorted(self.last_message_ns.items())]
        self.get_logger().info("Rerun summary: %s; last_ns: %s" % (
            ", ".join(parts) or "no messages", ", ".join(latest) or "none"))
        self.session.close()
        return super().destroy_node()


def main(args: Optional[Any] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RerunBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
