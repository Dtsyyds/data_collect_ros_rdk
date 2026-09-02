"""TF validation and logging."""

import math
from typing import Any, Callable

from .rerun_session import transform_entity


def log_tf_message(session: Any, message: Any, is_static: bool,
                   warn: Callable[[str], None]) -> int:
    logged = 0
    for transform in message.transforms:
        parent = str(transform.header.frame_id).strip().lstrip("/")
        child = str(transform.child_frame_id).strip().lstrip("/")
        q = transform.transform.rotation
        values = [float(q.x), float(q.y), float(q.z), float(q.w)]
        norm = math.sqrt(sum(value * value for value in values))
        if not parent or not child:
            warn("TF skipped because parent or child frame is empty")
            continue
        if not math.isfinite(norm) or norm < 1e-12:
            warn("TF %s -> %s skipped because quaternion is invalid" % (parent, child))
            continue
        values = [value / norm for value in values]
        translation = transform.transform.translation
        xyz = [float(translation.x), float(translation.y), float(translation.z)]
        if not all(math.isfinite(value) for value in xyz):
            warn("TF %s -> %s skipped because translation is invalid" % (parent, child))
            continue
        entity_path = "world/frames/%s" % child.replace("/", "_")
        session.log(entity_path, transform_entity(xyz, values, parent, child), static=is_static)
        logged += 1
    return logged
