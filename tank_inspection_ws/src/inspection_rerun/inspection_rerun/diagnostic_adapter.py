"""DiagnosticArray compatibility helpers."""

from typing import Any


def diagnostic_level(value: Any) -> int:
    """Normalize ROS uint8 representations used by different Python generators."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError("diagnostic level byte field must have length one")
        return int(value[0])
    return int(value)
