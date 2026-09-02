import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from inspection_rerun.pointcloud_adapter import extract_pointcloud


def make_cloud(points):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(Header(frame_id="lidar"), fields, points)


def test_fields_and_intensity_are_parsed():
    result = extract_pointcloud(make_cloud([(1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 7.0, 8.0)]))
    assert result.positions.tolist() == [[1.0, 2.0, 3.0], [5.0, 6.0, 7.0]]
    assert result.colors.shape == (2, 3)


def test_nan_and_inf_are_filtered():
    cloud = make_cloud([(1.0, 2.0, 3.0, 1.0), (math.nan, 0.0, 0.0, 2.0),
                        (0.0, math.inf, 0.0, 3.0)])
    result = extract_pointcloud(cloud)
    assert result.positions.shape == (1, 3)


def test_deterministic_max_count_and_stride():
    cloud = make_cloud([(float(i), 0.0, 0.0, float(i)) for i in range(20)])
    one = extract_pointcloud(cloud, stride=2, max_points=4).positions
    two = extract_pointcloud(cloud, stride=2, max_points=4).positions
    np.testing.assert_array_equal(one, two)
    assert len(one) == 4
