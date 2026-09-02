"""The only module that owns Rerun initialization and connection APIs."""

from pathlib import Path
from typing import Any, Optional

import numpy as np
import rerun as rr


class RerunSession:
    def __init__(self, output_mode: str, recording_id: str, save_path: str,
                 connect_url: str = "rerun+http://127.0.0.1:9876/proxy",
                 blueprint: Optional[Any] = None) -> None:
        rr.init("tank_inspection", recording_id=recording_id,
                default_blueprint=blueprint)
        if output_mode == "spawn":
            rr.spawn()
        elif output_mode == "connect":
            rr.connect_grpc(connect_url)
        elif output_mode == "save":
            path = Path(save_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            rr.save(path, default_blueprint=blueprint)
        else:
            raise ValueError("unsupported output mode: %s" % output_mode)
        self.output_mode = output_mode

    def set_time(self, ros_time_ns: int, sequence: int) -> None:
        # Rerun 0.36 interprets a bare Python int as seconds. Keep the bridge's
        # canonical representation as int nanoseconds and make the unit explicit
        # only at this SDK boundary.
        rr.set_time("ros_time", timestamp=np.datetime64(int(ros_time_ns), "ns"))
        rr.set_time("frame_sequence", sequence=int(sequence))

    def log(self, entity_path: str, *entities: Any, static: bool = False) -> None:
        rr.log(entity_path, *entities, static=static)

    def close(self) -> None:
        rr.disconnect()


def image_entity(image: Any, depth_units_per_meter: Optional[float] = None) -> Any:
    if depth_units_per_meter is not None:
        return rr.DepthImage(image, meter=float(depth_units_per_meter), colormap="viridis")
    return rr.Image(image)


def points3d_entity(points: Any, colors: Optional[Any] = None) -> Any:
    if colors is None:
        return rr.Points3D(points, colors=[80, 180, 255])
    return rr.Points3D(points, colors=colors)


def coordinate_frame_entity(frame_id: str) -> Any:
    frame = str(frame_id).strip().lstrip("/")
    if not frame:
        raise ValueError("coordinate frame id cannot be empty")
    return rr.CoordinateFrame(frame)


def scalar_entity(value: Any) -> Any:
    return rr.Scalars([float(value)])


def text_entity(text: str, level: str = "INFO") -> Any:
    return rr.TextLog(text, level=level)


def tensor_entity(values: Any, dim_names: Any) -> Any:
    return rr.Tensor(values, dim_names=dim_names)


def transform_entity(translation: Any, quaternion_xyzw: Any, parent: str, child: str) -> Any:
    return rr.Transform3D(
        translation=translation,
        quaternion=rr.Quaternion(xyzw=quaternion_xyzw),
        parent_frame=parent,
        child_frame=child,
    )


def pinhole_entity(focal_length: Any, principal_point: Any, resolution: Any,
                   frame_id: str) -> Any:
    return rr.Pinhole(
        focal_length=focal_length,
        principal_point=principal_point,
        resolution=resolution,
        # The ROS optical frame is the 3D parent of the pinhole image plane.
        # Treating it as the child gives it a second parent beside base_link.
        parent_frame=frame_id or "camera",
    )
