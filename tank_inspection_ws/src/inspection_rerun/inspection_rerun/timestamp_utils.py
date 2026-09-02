"""Integer-nanosecond timestamp helpers and deterministic rate limiting."""

from dataclasses import dataclass
from typing import Any, Optional, Tuple

NSEC_PER_SEC = 1_000_000_000


def stamp_to_ns(stamp: Any) -> int:
    """Convert a ROS-like builtin_interfaces/Time to integer nanoseconds."""
    sec = int(stamp.sec)
    nanosec = int(stamp.nanosec)
    if nanosec < 0 or nanosec >= NSEC_PER_SEC:
        raise ValueError("nanosec must be in [0, 1000000000)")
    return sec * NSEC_PER_SEC + nanosec


def message_time_ns(message: Any, receive_time_ns: int) -> Tuple[int, bool]:
    """Return (time_ns, used_receive_time), falling back on missing/zero headers."""
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is None:
        return int(receive_time_ns), True
    value = stamp_to_ns(stamp)
    if value == 0:
        return int(receive_time_ns), True
    return value, False


@dataclass
class TimelineState:
    sequence: int = 0
    last_time_ns: Optional[int] = None
    rollback_count: int = 0

    def advance(self, time_ns: int) -> Tuple[int, bool]:
        rolled_back = self.last_time_ns is not None and time_ns < self.last_time_ns
        if rolled_back:
            self.rollback_count += 1
        self.last_time_ns = int(time_ns)
        current = self.sequence
        self.sequence += 1
        return current, rolled_back


class RateLimiter:
    """Timestamp-based limiter that safely resets when bag time moves backwards."""

    def __init__(self, max_hz: float) -> None:
        if max_hz <= 0.0:
            raise ValueError("max_hz must be positive")
        self.period_ns = max(1, int(NSEC_PER_SEC / float(max_hz)))
        self.last_accepted_ns = None  # type: Optional[int]
        self.accepted = 0
        self.skipped = 0

    def allow(self, time_ns: int) -> bool:
        value = int(time_ns)
        if self.last_accepted_ns is None or value < self.last_accepted_ns:
            self.last_accepted_ns = value
            self.accepted += 1
            return True
        if value - self.last_accepted_ns < self.period_ns:
            self.skipped += 1
            return False
        self.last_accepted_ns = value
        self.accepted += 1
        return True
