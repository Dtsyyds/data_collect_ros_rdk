import pytest

from inspection_rerun.diagnostic_adapter import diagnostic_level


def test_diagnostic_level_accepts_int_and_ros_byte():
    assert diagnostic_level(2) == 2
    assert diagnostic_level(b"\x01") == 1


def test_diagnostic_level_rejects_bad_byte_length():
    with pytest.raises(ValueError):
        diagnostic_level(b"")
