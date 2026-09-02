"""Safe sensor_msgs/Image conversion with row-padding support."""

from typing import Any, Dict, Tuple

import numpy as np

_ENCODINGS = {
    "mono8": (np.dtype("u1"), 1, False),
    "mono16": (np.dtype("u2"), 1, False),
    "16uc1": (np.dtype("u2"), 1, False),
    "rgb8": (np.dtype("u1"), 3, False),
    "bgr8": (np.dtype("u1"), 3, True),
    "rgba8": (np.dtype("u1"), 4, False),
    "bgra8": (np.dtype("u1"), 4, True),
}  # type: Dict[str, Tuple[np.dtype, int, bool]]


def is_depth_encoding(encoding: str) -> bool:
    """Return whether a ROS image contains unsigned 16-bit depth samples."""
    return str(encoding).lower() == "16uc1"


def image_to_numpy(message: Any) -> np.ndarray:
    encoding = str(message.encoding).lower()
    if encoding not in _ENCODINGS:
        raise ValueError("unsupported image encoding: %s" % message.encoding)
    dtype, channels, swap_rb = _ENCODINGS[encoding]
    height, width, step = int(message.height), int(message.width), int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    itemsize = dtype.itemsize
    row_bytes = width * channels * itemsize
    if step < row_bytes:
        raise ValueError("image step is smaller than packed row size")
    raw = memoryview(message.data).cast("B")
    expected = step * height
    if len(raw) != expected:
        raise ValueError("image data length %d does not equal step*height %d" % (len(raw), expected))

    byte_rows = np.frombuffer(raw, dtype=np.uint8).reshape(height, step)
    packed = np.ascontiguousarray(byte_rows[:, :row_bytes])
    if itemsize == 2:
        byteorder = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
        array = packed.view(np.dtype(byteorder + "u2"))
        array = array.astype(np.uint16, copy=False)
    else:
        array = packed.view(dtype)
    if channels == 1:
        return array.reshape(height, width)
    array = array.reshape(height, width, channels)
    if swap_rb:
        if channels == 3:
            array = array[:, :, [2, 1, 0]]
        else:
            array = array[:, :, [2, 1, 0, 3]]
    return np.ascontiguousarray(array)
