from types import SimpleNamespace

from inspection_tools.depth_to_pointcloud import image_to_depth_array, project_depth_to_xyz
import numpy as np
import pytest


def test_project_depth_to_xyz_uses_optical_frame_convention():
    depth = np.array([[1000, 2000], [0, 3000]], dtype=np.uint16)

    xyz = project_depth_to_xyz(
        depth, fx=2.0, fy=2.0, cx=0.0, cy=0.0,
        depth_scale=0.001, stride=1, min_depth_m=0.05, max_depth_m=4.0)

    assert xyz.dtype == np.dtype("float32")
    assert xyz == pytest.approx(np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 2.0],
        [1.5, 1.5, 3.0],
    ], dtype=np.float32))


def test_project_depth_to_xyz_stride_uses_original_pixel_coordinates():
    depth = np.full((3, 3), 1000, dtype=np.uint16)

    xyz = project_depth_to_xyz(
        depth, fx=2.0, fy=2.0, cx=0.0, cy=0.0,
        stride=2, min_depth_m=0.0, max_depth_m=2.0)

    assert xyz == pytest.approx(np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
    ], dtype=np.float32))


def test_image_to_depth_array_removes_row_padding():
    pixels = np.array([[1, 2, 999], [3, 4, 999]], dtype="<u2")
    message = SimpleNamespace(
        encoding="16UC1",
        height=2,
        width=2,
        step=6,
        is_bigendian=False,
        data=pixels.tobytes(),
    )

    result = image_to_depth_array(message)

    assert result.tolist() == [[1, 2], [3, 4]]


@pytest.mark.parametrize("stride", [0, -1])
def test_project_depth_to_xyz_rejects_invalid_stride(stride):
    with pytest.raises(ValueError, match="stride"):
        project_depth_to_xyz(
            np.ones((1, 1), dtype=np.uint16),
            fx=1.0, fy=1.0, cx=0.0, cy=0.0, stride=stride)
