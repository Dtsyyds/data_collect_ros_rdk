"""Reconstruct a lightweight XYZ PointCloud2 from a recorded depth image.

This node is intentionally a replay/inspection utility.  The raw depth image and
camera calibration remain the source data in the bag; this derived cloud is not
recorded by the default hardware pipeline.
"""

from __future__ import annotations

import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def project_depth_to_xyz(
        depth: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        depth_scale: float = 0.001,
        stride: int = 2,
        min_depth_m: float = 0.05,
        max_depth_m: float = 10.0) -> np.ndarray:
    """Project a uint16 depth image into compact XYZ rows in the optical frame."""
    if depth.ndim != 2:
        raise ValueError("depth must be a two-dimensional array")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("fx and fy must be positive")
    if depth_scale <= 0.0:
        raise ValueError("depth_scale must be positive")
    if stride <= 0:
        raise ValueError("stride must be positive")
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("depth range must satisfy 0 <= min_depth_m < max_depth_m")

    sampled = depth[::stride, ::stride].astype(np.float32) * np.float32(depth_scale)
    v = np.arange(0, depth.shape[0], stride, dtype=np.float32)[:, None]
    u = np.arange(0, depth.shape[1], stride, dtype=np.float32)[None, :]
    valid = np.logical_and.reduce((
        np.isfinite(sampled),
        sampled >= np.float32(min_depth_m),
        sampled <= np.float32(max_depth_m),
    ))
    x = (u - np.float32(cx)) * sampled / np.float32(fx)
    y = (v - np.float32(cy)) * sampled / np.float32(fy)
    xyz = np.column_stack((x[valid], y[valid], sampled[valid]))
    return np.ascontiguousarray(xyz, dtype="<f4")


def image_to_depth_array(message: Image) -> np.ndarray:
    """Return the visible uint16 pixels while respecting byte order and row padding."""
    if message.encoding not in {"16UC1", "mono16"}:
        raise ValueError(
            f"unsupported depth encoding {message.encoding!r}; expected 16UC1 or mono16")
    if message.height <= 0 or message.width <= 0:
        raise ValueError("depth image dimensions must be positive")
    if message.step < message.width * 2 or message.step % 2:
        raise ValueError("depth image step is invalid for uint16 pixels")
    expected_bytes = int(message.height) * int(message.step)
    if len(message.data) != expected_bytes:
        raise ValueError(
            f"depth image data length {len(message.data)} != height*step {expected_bytes}")

    dtype = ">u2" if message.is_bigendian else "<u2"
    row_elements = int(message.step) // 2
    pixels = np.frombuffer(message.data, dtype=dtype).reshape(
        int(message.height), row_elements)
    return pixels[:, :int(message.width)]


class DepthToPointCloud(Node):
    """Convert recorded D405 depth frames to a downsampled, uncoloured XYZ cloud."""

    def __init__(self) -> None:
        super().__init__("depth_to_pointcloud")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter("output_topic", "/camera/reconstructed/depth/points")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("stride", 2)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 10.0)

        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._stride = int(self.get_parameter("stride").value)
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        # Validate parameters before subscriptions begin.
        project_depth_to_xyz(
            np.zeros((1, 1), dtype=np.uint16),
            1.0, 1.0, 0.0, 0.0,
            self._depth_scale, self._stride, self._min_depth_m, self._max_depth_m)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._camera_info: CameraInfo | None = None
        self._warned_missing_info = False
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), output_qos)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            input_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            input_qos,
        )
        self.get_logger().info(
            "D405 replay projector ready: stride=%d, depth_scale=%.6f m/unit, output=%s"
            % (
                self._stride,
                self._depth_scale,
                str(self.get_parameter("output_topic").value),
            )
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        if len(message.k) != 9 or message.k[0] <= 0.0 or message.k[4] <= 0.0:
            self.get_logger().warning("Ignoring CameraInfo with invalid intrinsic matrix")
            return
        self._camera_info = message

    def _on_depth(self, message: Image) -> None:
        if self._camera_info is None:
            if not self._warned_missing_info:
                self.get_logger().warning(
                    "Waiting for D405 CameraInfo before projecting depth frames")
                self._warned_missing_info = True
            return
        info = self._camera_info
        try:
            depth = image_to_depth_array(message)
            xyz = project_depth_to_xyz(
                depth,
                float(info.k[0]),
                float(info.k[4]),
                float(info.k[2]),
                float(info.k[5]),
                self._depth_scale,
                self._stride,
                self._min_depth_m,
                self._max_depth_m,
            )
        except ValueError as exc:
            self.get_logger().error(f"Cannot reconstruct depth frame: {exc}")
            return
        if xyz.shape[0] == 0:
            return

        cloud = PointCloud2()
        cloud.header = message.header
        cloud.height = 1
        cloud.width = int(xyz.shape[0])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = xyz.tobytes()
        cloud.is_dense = True
        self._publisher.publish(cloud)


def main(args=None) -> int:
    rclpy.init(args=args)
    try:
        node = DepthToPointCloud()
    except Exception as exc:
        rclpy.try_shutdown()
        print(f"depth_to_pointcloud configuration error: {exc}", file=sys.stderr)
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
