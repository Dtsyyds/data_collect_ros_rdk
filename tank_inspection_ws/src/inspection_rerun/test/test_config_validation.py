import pytest

from inspection_rerun.config_validation import validate_parameters


def valid():
    return {"output_mode": "save", "save_path": "out.rrd", "image_max_hz": 2.0,
            "pointcloud_max_hz": 1.0, "imu_max_hz": 20.0, "max_point_count": 100,
            "point_stride": 1, "point_voxel_size": 0.0}


def test_valid_parameters():
    validate_parameters(valid())


@pytest.mark.parametrize("field,value", [
    ("image_max_hz", 0.0), ("point_stride", 0), ("max_point_count", 0),
    ("point_voxel_size", -0.1), ("output_mode", "invalid")])
def test_invalid_parameters(field, value):
    values = valid()
    values[field] = value
    with pytest.raises(ValueError):
        validate_parameters(values)


def test_tank_scene_requires_both_files():
    values = valid()
    values.update({"log_tank_scene": True, "tank_asset_path": "tank.yaml",
                   "mission_path": ""})
    with pytest.raises(ValueError, match="mission_path"):
        validate_parameters(values)


@pytest.mark.parametrize("field,value", [
    ("fusion_height_field", "u_m"),
    ("actual_trajectory_min_step_m", -0.1),
    ("actual_trajectory_max_segments", 0),
    ("actual_trajectory_max_step_m", 0.0),
    ("depth_units_per_meter", 0.0),
])
def test_invalid_trajectory_parameters(field, value):
    values = valid()
    values[field] = value
    with pytest.raises(ValueError):
        validate_parameters(values)
