"""Pure parameter validation shared by the node and tests."""

from typing import Mapping, Any


def validate_parameters(values: Mapping[str, Any]) -> None:
    if values["output_mode"] not in ("spawn", "connect", "save"):
        raise ValueError("output_mode must be spawn, connect, or save")
    for name in ("image_max_hz", "pointcloud_max_hz", "imu_max_hz"):
        if float(values[name]) <= 0.0:
            raise ValueError("%s must be positive" % name)
    if float(values.get("depth_units_per_meter", 1000.0)) <= 0.0:
        raise ValueError("depth_units_per_meter must be positive")
    if int(values["max_point_count"]) < 1:
        raise ValueError("max_point_count must be at least 1")
    if int(values["point_stride"]) < 1:
        raise ValueError("point_stride must be at least 1")
    if float(values.get("point_voxel_size", 0.0)) < 0.0:
        raise ValueError("point_voxel_size cannot be negative")
    if values["output_mode"] == "save" and not str(values["save_path"]).strip():
        raise ValueError("save_path is required in save mode")
    if bool(values.get("log_tank_scene", False)):
        if not str(values.get("tank_asset_path", "")).strip():
            raise ValueError("tank_asset_path is required when log_tank_scene is enabled")
        if not str(values.get("mission_path", "")).strip():
            raise ValueError("mission_path is required when log_tank_scene is enabled")
    if str(values.get("fusion_height_field", "v_m")) != "v_m":
        raise ValueError("fusion_height_field must be v_m")
    if float(values.get("actual_trajectory_min_step_m", 0.01)) < 0.0:
        raise ValueError("actual_trajectory_min_step_m cannot be negative")
    if int(values.get("actual_trajectory_max_segments", 20000)) < 1:
        raise ValueError("actual_trajectory_max_segments must be at least 1")
    if float(values.get("actual_trajectory_max_step_m", 1.0)) <= 0.0:
        raise ValueError("actual_trajectory_max_step_m must be positive")
