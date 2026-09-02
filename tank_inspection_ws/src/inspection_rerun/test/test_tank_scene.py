import json
import math
from types import SimpleNamespace

import pytest
import yaml

from inspection_rerun.tank_scene import (fusion_observation, load_tank_scene,
                                          normalize_s, sz_to_tank_xyz,
                                          trajectory_strips)


def write_scene(tmp_path, mission_updates=None):
    asset = {
        "schema_version": "1.0", "asset_version": "asset-v1", "tank_id": "tank-1",
        "radius_m": 2.0, "height_m": 4.0, "start_angle_rad": 0.25,
        "warning": "simulation only",
        "courses": [{"course_id": "C01", "z_min_m": 0.0, "z_max_m": 4.0,
                     "seam_offset_m": 1.0, "plate_count": 4}],
    }
    mission = {
        "tank_id": "tank-1", "asset_version": "asset-v1", "radius_m": 2.0,
        "height_m": 4.0, "circumference_m": 4.0 * math.pi,
        "start_angle_rad": 0.25, "mission_id": "mission-1", "route_segments": [],
    }
    mission.update(mission_updates or {})
    asset_path = tmp_path / "tank_schematic.yaml"
    mission_path = tmp_path / "mission_schematic.json"
    asset_path.write_text(yaml.safe_dump(asset), encoding="utf-8")
    mission_path.write_text(json.dumps(mission), encoding="utf-8")
    return load_tank_scene(str(asset_path), str(mission_path))


def message(**updates):
    values = {"pose_valid": True, "asset_coordinate_valid": True, "s_m": 1.0,
              "v_m": 2.0, "n_m": 0.1, "pose_confidence": 0.8}
    values.update(updates)
    return SimpleNamespace(**values)


def test_loads_matching_scene_and_generates_plates(tmp_path):
    scene = write_scene(tmp_path)
    assert scene.tank_id == "tank-1"
    assert len(scene.plates) == 4
    assert scene.plates[0].s_start_m == pytest.approx(1.0)
    assert sum(plate.width_m for plate in scene.plates) == pytest.approx(scene.circumference_m)


def test_rejects_mission_geometry_mismatch(tmp_path):
    with pytest.raises(ValueError, match="radius_m does not match"):
        write_scene(tmp_path, {"radius_m": 3.0})


def test_sz_mapping_and_fusion_validation(tmp_path):
    scene = write_scene(tmp_path)
    observation = fusion_observation(message(), scene)
    expected = sz_to_tank_xyz(1.0, 2.0, 2.0, 0.25, 0.1)
    assert observation.xyz == pytest.approx(expected)
    assert normalize_s(scene.circumference_m + 1.0, scene.circumference_m) == pytest.approx(1.0)
    with pytest.raises(ValueError, match="invalid"):
        fusion_observation(message(pose_valid=False), scene)
    with pytest.raises(ValueError, match="outside"):
        fusion_observation(message(v_m=5.0), scene)


def test_unwrapped_trajectory_splits_at_seam(tmp_path):
    scene = write_scene(tmp_path)
    first = fusion_observation(message(s_m=scene.circumference_m - 0.1), scene)
    second = fusion_observation(message(s_m=0.1), scene)
    strips_3d, strips_2d = trajectory_strips(scene, first, second)
    assert len(strips_3d) == len(strips_2d) == 2
    assert strips_2d[0][-1][0] == pytest.approx(scene.circumference_m)
    assert strips_2d[1][0][0] == pytest.approx(0.0)
