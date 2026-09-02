"""PointCloud2 extraction using the ROS 2 official sensor_msgs_py helper."""

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

try:
    from sensor_msgs_py import point_cloud2
except ImportError as exc:  # pragma: no cover - environment-specific dependency error
    point_cloud2 = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


@dataclass
class PointCloudData:
    positions: np.ndarray
    colors: Optional[np.ndarray]


def _rgb_colors(values: np.ndarray) -> np.ndarray:
    if values.dtype.kind == "f":
        packed = np.asarray(values, dtype=np.float32).view(np.uint32)
    else:
        packed = np.asarray(values, dtype=np.uint32)
    return np.column_stack(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255)).astype(np.uint8)


def extract_pointcloud(message: Any, stride: int = 1, max_points: int = 100000,
                       voxel_size: float = 0.0) -> PointCloudData:
    if point_cloud2 is None:
        raise RuntimeError("sensor_msgs_py.point_cloud2 is required: %s" % _IMPORT_ERROR)
    if stride < 1 or max_points < 1 or voxel_size < 0.0:
        raise ValueError("stride/max_points/voxel_size parameters are invalid")
    available = {field.name for field in message.fields}
    if not {"x", "y", "z"}.issubset(available):
        raise ValueError("PointCloud2 must contain x, y and z fields")
    optional = "rgb" if "rgb" in available else ("intensity" if "intensity" in available else None)
    names = ["x", "y", "z"] + ([optional] if optional else [])
    records = point_cloud2.read_points(message, field_names=names, skip_nans=False)
    records = np.asarray(records).reshape(-1)
    positions = np.column_stack((records["x"], records["y"], records["z"])).astype(np.float32, copy=False)
    finite = np.isfinite(positions).all(axis=1)
    positions = positions[finite]
    colors = None
    if optional == "rgb":
        colors = _rgb_colors(records["rgb"][finite])
    elif optional == "intensity":
        intensity = np.asarray(records["intensity"][finite], dtype=np.float32)
        finite_i = np.isfinite(intensity)
        if np.any(finite_i):
            lo, hi = np.percentile(intensity[finite_i], [1.0, 99.0])
            scale = max(float(hi - lo), 1e-12)
            gray = np.clip((intensity - lo) / scale * 255.0, 0, 255).astype(np.uint8)
            colors = np.repeat(gray[:, None], 3, axis=1)

    indexes = np.arange(len(positions))[::stride]
    if voxel_size > 0.0 and len(indexes):
        voxels = np.floor(positions[indexes] / voxel_size).astype(np.int64)
        _, first = np.unique(voxels, axis=0, return_index=True)
        indexes = indexes[np.sort(first)]
    if len(indexes) > max_points:
        indexes = indexes[np.linspace(0, len(indexes) - 1, max_points, dtype=np.int64)]
    return PointCloudData(positions[indexes], colors[indexes] if colors is not None else None)
