"""CameraInfo validation and Rerun pinhole construction."""

import math
from typing import Any, Optional

from .rerun_session import pinhole_entity


def camera_info_entity(message: Any) -> Optional[Any]:
    k = list(message.k)
    if len(k) != 9 or not all(math.isfinite(float(value)) for value in k):
        raise ValueError("CameraInfo.k must contain nine finite values")
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0.0 or fy <= 0.0:
        return None
    return pinhole_entity([fx, fy], [cx, cy], [int(message.width), int(message.height)],
                          str(message.header.frame_id))
