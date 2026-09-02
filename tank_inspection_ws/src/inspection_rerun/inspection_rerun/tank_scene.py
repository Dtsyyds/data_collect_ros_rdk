"""Pure tank/mission loading and cylindrical coordinate helpers.

This module intentionally has no dependency on Rerun, ROS, OpenCV, or the
separate ``tank_vision`` checkout.  The bridge can therefore use the ROS Jazzy
Python 3.12 interpreter while consuming the same schematic file contract.
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import yaml


Point2D = Tuple[float, float]
Point3D = Tuple[float, float, float]


@dataclass(frozen=True)
class Course:
    course_id: str
    z_min_m: float
    z_max_m: float
    seam_offset_m: float
    plate_widths_m: Tuple[float, ...]


@dataclass(frozen=True)
class Plate:
    plate_id: str
    course_id: str
    s_start_m: float
    width_m: float
    z_min_m: float
    z_max_m: float


@dataclass(frozen=True)
class TankScene:
    tank_id: str
    asset_version: str
    radius_m: float
    height_m: float
    start_angle_rad: float
    warning: str
    courses: Tuple[Course, ...]
    plates: Tuple[Plate, ...]
    mission: Mapping[str, Any]

    @property
    def circumference_m(self) -> float:
        return 2.0 * math.pi * self.radius_m


@dataclass(frozen=True)
class FusionObservation:
    s_m: float
    z_m: float
    normal_m: float
    xyz: Point3D


def _finite_number(value: Any, owner: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("%s must be a number" % owner)
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("%s must be finite" % owner)
    return result


def _mapping(value: Any, owner: str) -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("%s must be a mapping" % owner)
    return dict(value)


def _sequence(value: Any, owner: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError("%s must be a list" % owner)
    return value


def normalize_s(s_m: float, circumference_m: float) -> float:
    s_m = _finite_number(s_m, "s_m")
    circumference_m = _finite_number(circumference_m, "circumference_m")
    if circumference_m <= 0.0:
        raise ValueError("circumference_m must be positive")
    result = s_m % circumference_m
    if math.isclose(result, circumference_m, rel_tol=0.0, abs_tol=1e-12):
        return 0.0
    return result


def sz_to_tank_xyz(s_m: float, z_m: float, radius_m: float,
                   start_angle_rad: float = 0.0, normal_m: float = 0.0) -> Point3D:
    """Map unwrapped wall ``(s,z)`` plus outward normal offset to tank XYZ."""
    radius_m = _finite_number(radius_m, "radius_m")
    z_m = _finite_number(z_m, "z_m")
    start_angle_rad = _finite_number(start_angle_rad, "start_angle_rad")
    normal_m = _finite_number(normal_m, "normal_m")
    if radius_m <= 0.0 or radius_m + normal_m <= 0.0:
        raise ValueError("tank radius and offset radius must be positive")
    circumference = 2.0 * math.pi * radius_m
    theta = normalize_s(s_m, circumference) / radius_m
    display_radius = radius_m + normal_m
    angle = start_angle_rad + theta
    return (display_radius * math.cos(angle), display_radius * math.sin(angle), z_m)


def _load_courses(raw_courses: Any, circumference: float, height_m: float) -> Tuple[Course, ...]:
    courses = []  # type: List[Course]
    identifiers = set()
    for index, value in enumerate(_sequence(raw_courses, "courses")):
        data = _mapping(value, "courses[%d]" % index)
        course_id = data.get("course_id")
        if not isinstance(course_id, str) or not course_id or course_id in identifiers:
            raise ValueError("course_id must be non-empty and unique")
        identifiers.add(course_id)
        z_min = _finite_number(data.get("z_min_m"), "%s.z_min_m" % course_id)
        z_max = _finite_number(data.get("z_max_m"), "%s.z_max_m" % course_id)
        if z_min < 0.0 or z_max <= z_min or z_max > height_m + 1e-9:
            raise ValueError("%s has invalid height bounds" % course_id)
        offset = normalize_s(_finite_number(data.get("seam_offset_m", 0.0),
                                             "%s.seam_offset_m" % course_id), circumference)
        widths_raw = data.get("plate_widths_m")
        if widths_raw:
            widths = tuple(_finite_number(item, "%s.plate_widths_m" % course_id)
                           for item in _sequence(widths_raw, "%s.plate_widths_m" % course_id))
            if any(width <= 0.0 for width in widths):
                raise ValueError("plate widths must be positive")
            if not math.isclose(sum(widths), circumference, rel_tol=0.0, abs_tol=1e-6):
                raise ValueError("%s plate widths do not cover the circumference" % course_id)
        else:
            count = data.get("plate_count")
            if isinstance(count, bool) or not isinstance(count, int) or count < 1:
                raise ValueError("%s.plate_count must be a positive integer" % course_id)
            widths = (circumference / count,) * count
        courses.append(Course(course_id, z_min, z_max, offset, widths))
    if not courses:
        raise ValueError("at least one tank course is required")
    return tuple(courses)


def _build_plates(courses: Sequence[Course], circumference: float) -> Tuple[Plate, ...]:
    result = []  # type: List[Plate]
    for course in courses:
        cursor = course.seam_offset_m
        for index, width in enumerate(course.plate_widths_m):
            result.append(Plate(
                "%s-P%03d" % (course.course_id, index + 1), course.course_id,
                normalize_s(cursor, circumference), width, course.z_min_m, course.z_max_m))
            cursor += width
    return tuple(result)


def load_tank_scene(asset_path: str, mission_path: str) -> TankScene:
    """Load and cross-check ``tank_schematic.yaml`` and mission JSON."""
    asset_file = Path(asset_path).expanduser()
    mission_file = Path(mission_path).expanduser()
    if not asset_file.is_file():
        raise FileNotFoundError("tank schematic not found: %s" % asset_file)
    if not mission_file.is_file():
        raise FileNotFoundError("mission schematic not found: %s" % mission_file)
    with asset_file.open("r", encoding="utf-8") as handle:
        asset = _mapping(yaml.safe_load(handle), "tank schematic")
    with mission_file.open("r", encoding="utf-8") as handle:
        mission = _mapping(json.load(handle), "mission schematic")

    tank_id = asset.get("tank_id")
    asset_version = asset.get("asset_version")
    if not isinstance(tank_id, str) or not tank_id:
        raise ValueError("tank schematic tank_id is required")
    if not isinstance(asset_version, str) or not asset_version:
        raise ValueError("tank schematic asset_version is required")
    radius = _finite_number(asset.get("radius_m"), "tank schematic radius_m")
    height = _finite_number(asset.get("height_m"), "tank schematic height_m")
    start_angle = _finite_number(asset.get("start_angle_rad", 0.0),
                                 "tank schematic start_angle_rad")
    if radius <= 0.0 or height <= 0.0:
        raise ValueError("tank radius and height must be positive")
    circumference = 2.0 * math.pi * radius
    courses = _load_courses(asset.get("courses"), circumference, height)

    pairs = (("tank_id", tank_id), ("asset_version", asset_version))
    for key, expected in pairs:
        if mission.get(key) != expected:
            raise ValueError("mission %s does not match tank schematic" % key)
    for key, expected in (("radius_m", radius), ("height_m", height),
                          ("circumference_m", circumference),
                          ("start_angle_rad", start_angle)):
        actual = _finite_number(mission.get(key), "mission %s" % key)
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("mission %s does not match tank schematic" % key)
    if not isinstance(mission.get("route_segments", []), list):
        raise ValueError("mission route_segments must be a list")

    warning = asset.get("warning", "")
    if not isinstance(warning, str):
        raise ValueError("tank schematic warning must be a string")
    return TankScene(tank_id, asset_version, radius, height, start_angle, warning,
                     courses, _build_plates(courses, circumference), mission)


def path_3d(scene: TankScene, path: Sequence[Point2D], normal_m: float = 0.0) -> List[Point3D]:
    """Densely sample an SZ polyline so cylinder arcs do not appear as chords."""
    sampled = []  # type: List[Point3D]
    for first, second in zip(path, path[1:]):
        delta_s = second[0] - first[0]
        delta_z = second[1] - first[1]
        count = max(1, int(math.ceil(abs(delta_s) / scene.circumference_m * 128.0)),
                    int(math.ceil(abs(delta_z))))
        for index in range(count + 1):
            if sampled and index == 0:
                continue
            fraction = float(index) / count
            sampled.append(sz_to_tank_xyz(first[0] + delta_s * fraction,
                                           first[1] + delta_z * fraction,
                                           scene.radius_m, scene.start_angle_rad, normal_m))
    if not sampled and path:
        sampled.append(sz_to_tank_xyz(path[0][0], path[0][1], scene.radius_m,
                                      scene.start_angle_rad, normal_m))
    return sampled


def plate_outlines_2d(plate: Plate, circumference: float) -> List[List[Point2D]]:
    end = plate.s_start_m + plate.width_m
    ranges = [(plate.s_start_m, min(end, circumference))]
    if end > circumference + 1e-9:
        ranges.append((0.0, end - circumference))
    return [[(start, plate.z_min_m), (finish, plate.z_min_m),
             (finish, plate.z_max_m), (start, plate.z_max_m),
             (start, plate.z_min_m)]
            for start, finish in ranges if finish - start > 1e-9]


def plate_outline_3d(scene: TankScene, plate: Plate) -> List[Point3D]:
    count = max(4, int(math.ceil(plate.width_m / scene.circumference_m * 128.0)))
    bottom = [plate.s_start_m + plate.width_m * index / count for index in range(count + 1)]
    path = ([(s_m, plate.z_min_m) for s_m in bottom] +
            [(s_m, plate.z_max_m) for s_m in reversed(bottom)] +
            [(plate.s_start_m, plate.z_min_m)])
    return [sz_to_tank_xyz(s_m, z_m, scene.radius_m, scene.start_angle_rad)
            for s_m, z_m in path]


def fusion_observation(message: Any, scene: TankScene,
                       height_field: str = "v_m") -> FusionObservation:
    """Validate and convert the canonical FusionIndex wall coordinate."""
    if height_field != "v_m":
        raise ValueError("fusion_height_field must be v_m")
    if not bool(message.pose_valid) or not bool(message.asset_coordinate_valid):
        raise ValueError("FusionIndex pose/asset coordinate is invalid")
    s_m = _finite_number(message.s_m, "FusionIndex.s_m")
    z_m = _finite_number(getattr(message, height_field), "FusionIndex.%s" % height_field)
    normal_m = _finite_number(message.n_m, "FusionIndex.n_m")
    if z_m < -1e-6 or z_m > scene.height_m + 1e-6:
        raise ValueError("FusionIndex height is outside the tank")
    s_normalized = normalize_s(s_m, scene.circumference_m)
    xyz = sz_to_tank_xyz(s_normalized, z_m, scene.radius_m,
                         scene.start_angle_rad, normal_m)
    return FusionObservation(s_normalized, z_m, normal_m, xyz)


def trajectory_strips(scene: TankScene, first: FusionObservation,
                      second: FusionObservation) -> Tuple[List[List[Point3D]], List[List[Point2D]]]:
    """Return matching 3D/2D strips, splitting a segment across the unwrap seam."""
    circumference = scene.circumference_m
    delta = second.s_m - first.s_m
    if abs(delta) <= circumference / 2.0:
        path = [(first.s_m, first.z_m), (second.s_m, second.z_m)]
        normal = 0.5 * (first.normal_m + second.normal_m)
        return ([path_3d(scene, path, normal)], [path])

    if delta < 0.0:
        second_unwrapped = second.s_m + circumference
        fraction = (circumference - first.s_m) / (second_unwrapped - first.s_m)
        boundary_z = first.z_m + (second.z_m - first.z_m) * fraction
        paths_2d = [[(first.s_m, first.z_m), (circumference, boundary_z)],
                    [(0.0, boundary_z), (second.s_m, second.z_m)]]
    else:
        first_unwrapped = first.s_m + circumference
        fraction = (circumference - second.s_m) / (first_unwrapped - second.s_m)
        boundary_z = second.z_m + (first.z_m - second.z_m) * fraction
        paths_2d = [[(first.s_m, first.z_m), (0.0, boundary_z)],
                    [(circumference, boundary_z), (second.s_m, second.z_m)]]
    normal = 0.5 * (first.normal_m + second.normal_m)
    return ([path_3d(scene, path, normal) for path in paths_2d], paths_2d)
