"""Odometry, Path and PoseStamped visualization helpers."""

from typing import Any, Iterable

import numpy as np

from .rerun_session import points3d_entity, transform_entity


def _pose_parts(pose: Any):
    position = pose.position
    orientation = pose.orientation
    return ([position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w])


def log_pose(session: Any, pose: Any, parent: str, child: str, entity_path: str) -> None:
    xyz, quat = _pose_parts(pose)
    session.log(entity_path, transform_entity(xyz, quat, parent or "world", child))


def log_path(session: Any, poses: Iterable[Any]) -> int:
    points = [[item.pose.position.x, item.pose.position.y, item.pose.position.z] for item in poses]
    if points:
        session.log("world/robot/path", points3d_entity(np.asarray(points, dtype=np.float32)))
    return len(points)
