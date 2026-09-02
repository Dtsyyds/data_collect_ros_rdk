from types import SimpleNamespace

from inspection_rerun.timestamp_utils import TimelineState, message_time_ns, stamp_to_ns


def stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_stamp_to_integer_nanoseconds():
    assert stamp_to_ns(stamp(7, 23)) == 7_000_000_023


def test_zero_stamp_uses_receive_time():
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(0, 0)))
    assert message_time_ns(message, 1234) == (1234, True)


def test_missing_header_uses_receive_time():
    assert message_time_ns(SimpleNamespace(), 5678) == (5678, True)


def test_timeline_accepts_time_rollback():
    timeline = TimelineState()
    assert timeline.advance(100) == (0, False)
    assert timeline.advance(90) == (1, True)
    assert timeline.rollback_count == 1
