"""IMU logging helpers."""

from typing import Any

from .rerun_session import scalar_entity


def log_imu(session: Any, message: Any, root: str = "signals/imu") -> None:
    linear = message.linear_acceleration
    angular = message.angular_velocity
    for axis in ("x", "y", "z"):
        session.log("%s/acceleration/%s" % (root, axis), scalar_entity(getattr(linear, axis)))
        session.log("%s/angular_velocity/%s" % (root, axis), scalar_entity(getattr(angular, axis)))
    covariance = list(message.orientation_covariance)
    if not covariance or covariance[0] != -1.0:
        orientation = message.orientation
        for axis in ("x", "y", "z", "w"):
            session.log("%s/orientation/%s" % (root, axis), scalar_entity(getattr(orientation, axis)))
