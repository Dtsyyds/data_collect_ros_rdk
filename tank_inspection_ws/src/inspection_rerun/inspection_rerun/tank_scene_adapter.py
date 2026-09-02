"""Rerun entities for a linked tank, mission, and actual FusionIndex path."""

import math
from collections import defaultdict
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rerun as rr
import rerun.blueprint as rrb

from .tank_scene import (FusionObservation, TankScene, fusion_observation,
                         path_3d, plate_outline_3d, plate_outlines_2d,
                         sz_to_tank_xyz, trajectory_strips)


COLORS = {
    "coverage": (110, 150, 180, 125),
    "thickness_approach": (210, 180, 95, 150),
    "plate_thickness": (210, 180, 95, 150),
    "weld": (170, 125, 190, 145),
    "weld_unreachable": (205, 100, 100, 165),
    "obstacle": (130, 135, 145, 130),
    "plate": (90, 105, 125, 160),
    "actual": (255, 190, 90, 235),
}


def build_tank_blueprint(scene: TankScene) -> Any:
    """Make 3D and unwrapped wall views share the same entity hierarchy."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/", contents="/**", name="Tank + actual trajectory",
                              background=(15, 20, 30), line_grid=True),
            rrb.Vertical(
                rrb.Spatial2DView(
                    origin="/", contents="/**", name="Unwrapped wall (s, z)",
                    visual_bounds=rrb.VisualBounds2D(
                        x_range=[0.0, scene.circumference_m],
                        y_range=[0.0, scene.height_m])),
                rrb.TextDocumentView(origin="/mission_info/summary", name="Mission status"),
                row_shares=[0.72, 0.28]),
            column_shares=[0.58, 0.42]),
        collapse_panels=False)


def _tank_mesh(scene: TankScene) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int, int]]]:
    count = 128
    # Use the physical wall radius. Display trajectories use UI-point radii,
    # so shrinking the mesh is unnecessary and visibly detached them by 3 cm.
    radius = scene.radius_m
    vertices = []
    for z_m in (0.0, scene.height_m):
        for index in range(count):
            angle = scene.start_angle_rad + 2.0 * math.pi * index / count
            vertices.append((radius * math.cos(angle), radius * math.sin(angle), z_m))
    triangles = []
    for index in range(count):
        next_index = (index + 1) % count
        triangles.extend(((index, next_index, count + index),
                          (next_index, count + next_index, count + index)))
    return vertices, triangles


def _route_path(segment: Dict[str, Any]) -> List[Tuple[float, float]]:
    points = []
    for waypoint in segment.get("waypoints", []):
        points.append((float(waypoint["s_m"]), float(waypoint["z_m"])))
    return points


def _log_route_group(session: Any, scene: TankScene, route_type: str,
                     segments: Sequence[Dict[str, Any]]) -> None:
    if not segments:
        return
    paths_2d = [_route_path(segment) for segment in segments]
    paths_2d = [path for path in paths_2d if path]
    if not paths_2d:
        return
    labels = [str(segment.get("segment_id", "")) for segment in segments if _route_path(segment)]
    color = COLORS.get(route_type, COLORS["coverage"])
    colors = [color] * len(paths_2d)
    session.log(
        "world/inspection/mission/%s" % route_type,
        rr.LineStrips3D([path_3d(scene, path) for path in paths_2d], colors=colors,
                        labels=labels, show_labels=False,
                        radii=rr.Radius.ui_points(1.5)),
        rr.LineStrips2D(paths_2d, colors=colors, labels=labels, show_labels=False,
                        radii=rr.Radius.ui_points(1.5)),
        rr.AnyValues(segment_id=labels, route_type=[route_type] * len(labels)),
        static=True)


def log_asset_context(session: Any, scene: TankScene) -> None:
    """Log the static tank shell and linked plate outlines."""
    # ROS TF uses tank_course as its root while scene entities inherit the
    # implicit /world frame. Join both roots with one static identity relation.
    session.log(
        "world/inspection/tank_frame",
        rr.Transform3D(parent_frame="tf#/world", child_frame="tank_course"),
        static=True)
    vertices, triangles = _tank_mesh(scene)
    session.log("world/inspection/asset_context/tank_shell",
                rr.Mesh3D(vertex_positions=vertices, triangle_indices=triangles,
                          albedo_factor=(45, 58, 75, 255)), static=True)
    instances = [(plate, outline) for plate in scene.plates
                 for outline in plate_outlines_2d(plate, scene.circumference_m)]
    labels = [plate.plate_id for plate, _ in instances]
    colors = [COLORS["plate"]] * len(instances)
    session.log(
        "world/inspection/asset_context/plates",
        rr.LineStrips3D([plate_outline_3d(scene, plate) for plate, _ in instances],
                        colors=colors, labels=labels, show_labels=False,
                        radii=rr.Radius.ui_points(0.75)),
        rr.LineStrips2D([outline for _, outline in instances], colors=colors,
                        labels=labels, show_labels=False,
                        radii=rr.Radius.ui_points(0.75)),
        rr.AnyValues(plate_id=labels,
                     course_id=[plate.course_id for plate, _ in instances]),
        static=True)


def _box_outline(box: Dict[str, Any]) -> List[Tuple[float, float]]:
    return [(float(box["s_min_m"]), float(box["z_min_m"])),
            (float(box["s_max_m"]), float(box["z_min_m"])),
            (float(box["s_max_m"]), float(box["z_max_m"])),
            (float(box["s_min_m"]), float(box["z_max_m"])),
            (float(box["s_min_m"]), float(box["z_min_m"]))]


def _summary_markdown(scene: TankScene) -> str:
    mission = scene.mission
    stats = mission.get("statistics", {})
    warning_lines = [scene.warning] + [str(item) for item in mission.get("warnings", [])]
    warnings = "\n".join("> **%s**" % item for item in warning_lines if item)
    return """# Mission: {mission_id}

{warnings}

- tank_id: `{tank_id}`
- planning_valid: `{planning_valid}`
- radius / height: `{radius:.3f} m` / `{height:.3f} m`
- courses / plates: `{courses}` / `{plates}`
- route segments: `{routes}`
- thickness tasks: `{thickness}`
- execution events: `{events}`
- actual timeline: `ros_time`
- planning preview timeline: `inspection_step` (not emitted by this live bridge)
""".format(
        mission_id=mission.get("mission_id", ""), warnings=warnings,
        tank_id=scene.tank_id, planning_valid=mission.get("planning_valid", ""),
        radius=scene.radius_m, height=scene.height_m, courses=len(scene.courses),
        plates=len(scene.plates), routes=len(mission.get("route_segments", [])),
        thickness=stats.get("thickness_point_total_count",
                            len(mission.get("thickness_tasks", []))),
        events=stats.get("execution_event_count", len(mission.get("execution_sequence", []))))


def log_mission(session: Any, scene: TankScene) -> None:
    """Log static planned routes, expanded obstacles, and thickness targets."""
    grouped = defaultdict(list)
    for segment in scene.mission.get("route_segments", []):
        if isinstance(segment, dict):
            grouped[str(segment.get("route_type", "other"))].append(segment)
    for route_type, segments in grouped.items():
        _log_route_group(session, scene, route_type, segments)

    obstacle_paths = []
    obstacle_labels = []
    for obstacle in scene.mission.get("expanded_obstacles", []):
        if not isinstance(obstacle, dict):
            continue
        for box in obstacle.get("boxes", []):
            obstacle_paths.append(_box_outline(box))
            obstacle_labels.append(str(obstacle.get("obstacle_id", "")))
    if obstacle_paths:
        colors = [COLORS["obstacle"]] * len(obstacle_paths)
        session.log(
            "world/inspection/mission/expanded_obstacles",
            rr.LineStrips3D([path_3d(scene, path) for path in obstacle_paths], colors=colors,
                            labels=obstacle_labels, show_labels=False,
                            radii=rr.Radius.ui_points(2.0)),
            rr.LineStrips2D(obstacle_paths, colors=colors, labels=obstacle_labels,
                            show_labels=False, radii=rr.Radius.ui_points(2.0)),
            rr.AnyValues(obstacle_id=obstacle_labels), static=True)

    task_groups = {"thickness_stops": [], "unavailable_thickness": []}
    for task in scene.mission.get("thickness_tasks", []):
        if not isinstance(task, dict) or not isinstance(task.get("target"), dict):
            continue
        name = ("thickness_stops" if task.get("status") in ("PLANNED", "SIMULATION_ONLY")
                else "unavailable_thickness")
        task_groups[name].append(task)
    for name, tasks in task_groups.items():
        if not tasks:
            continue
        positions_2d = [(float(task["target"]["s_m"]), float(task["target"]["z_m"]))
                        for task in tasks]
        positions_3d = [sz_to_tank_xyz(s_m, z_m, scene.radius_m, scene.start_angle_rad)
                        for s_m, z_m in positions_2d]
        labels = [str(task.get("point_id", task.get("task_id", ""))) for task in tasks]
        color = COLORS["thickness_approach"] if name == "thickness_stops" else COLORS["weld_unreachable"]
        session.log(
            "world/inspection/mission/%s" % name,
            rr.Points3D(positions_3d, colors=[color] * len(tasks), labels=labels,
                        show_labels=False, radii=rr.Radius.ui_points(2.75)),
            rr.Points2D(positions_2d, colors=[color] * len(tasks), labels=labels,
                        show_labels=False, radii=rr.Radius.ui_points(2.75)),
            rr.AnyValues(point_id=labels, status=[str(task.get("status", "")) for task in tasks]),
            static=True)
    session.log("mission_info/summary",
                rr.TextDocument(_summary_markdown(scene), media_type="text/markdown"),
                static=True)


class ActualTrajectoryLogger:
    """Append a bounded, time-aware actual route without re-logging its full history."""

    def __init__(self, scene: TankScene, min_step_m: float = 0.01,
                 max_segments: int = 20000, height_field: str = "v_m",
                 max_step_m: float = 1.0) -> None:
        self.scene = scene
        self.min_step_m = float(min_step_m)
        self.max_segments = int(max_segments)
        self.height_field = height_field
        self.max_step_m = float(max_step_m)
        self.previous = None  # type: Optional[FusionObservation]
        self.segment_count = 0

    def log(self, session: Any, message: Any) -> bool:
        observation = fusion_observation(message, self.scene, self.height_field)
        color = COLORS["actual"]
        session.log(
            "world/inspection/actual_position",
            rr.Points3D([observation.xyz], colors=[color],
                        radii=rr.Radius.ui_points(5.0)),
            rr.Points2D([(observation.s_m, observation.z_m)], colors=[color],
                        radii=rr.Radius.ui_points(5.0)),
            rr.AnyValues(s_m=[observation.s_m], z_m=[observation.z_m],
                         normal_m=[observation.normal_m],
                         pose_confidence=[float(message.pose_confidence)]))
        if self.previous is None:
            self.previous = observation
            return True
        ds = min(abs(observation.s_m - self.previous.s_m),
                 self.scene.circumference_m - abs(observation.s_m - self.previous.s_m))
        distance = math.hypot(ds, observation.z_m - self.previous.z_m)
        if distance > self.max_step_m:
            # A mission REPOSITION is a discontinuity, never a traversed path.
            self.previous = observation
            return True
        if distance < self.min_step_m:
            return True
        strips_3d, strips_2d = trajectory_strips(self.scene, self.previous, observation)
        slot = self.segment_count % self.max_segments
        session.log(
            "world/inspection/actual_trajectory/segments/%05d" % slot,
            rr.LineStrips3D(strips_3d, colors=[color] * len(strips_3d),
                            radii=rr.Radius.ui_points(2.5)),
            rr.LineStrips2D(strips_2d, colors=[color] * len(strips_2d),
                            radii=rr.Radius.ui_points(2.5)))
        self.segment_count += 1
        self.previous = observation
        return True
