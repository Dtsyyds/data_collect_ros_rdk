from types import SimpleNamespace

from inspection_tools.validate_bag import _uint8_value
from inspection_tools.validators import (
    validate_eddy_current,
    validate_image,
    validate_pointcloud2,
    validate_ultrasound,
)


def test_ros_uint8_normalization_accepts_integer_and_bytes():
    assert _uint8_value(1) == 1
    assert _uint8_value(b"\x02") == 2


def test_depth_and_color_image_validation():
    depth = SimpleNamespace(
        width=2, height=2, encoding="16UC1", step=4, data=bytes(8))
    color = SimpleNamespace(
        width=2, height=2, encoding="rgb8", step=6, data=bytes(12))
    assert validate_image(depth, {"16UC1", "mono16"}) == []
    assert validate_image(color, {"rgb8", "bgr8"}) == []

    color.step = 2
    color.data = bytes(4)
    errors = validate_image(color, {"rgb8", "bgr8"})
    assert any("minimum" in item for item in errors)

    color.encoding = "mono8"
    errors = validate_image(color, {"rgb8", "bgr8"})
    assert any("encoding" in item for item in errors)


def test_ultrasound_array_length_validation():
    valid = SimpleNamespace(channel_count=2, sample_count=4, samples=list(range(8)))
    invalid = SimpleNamespace(channel_count=2, sample_count=4, samples=list(range(7)))
    assert validate_ultrasound(valid) == []
    assert "channel_count*sample_count" in validate_ultrasound(invalid)[0]


def test_pointcloud2_field_validation():
    fields = [SimpleNamespace(name=name) for name in ("x", "y", "z", "intensity", "time_offset")]
    valid = SimpleNamespace(
        fields=fields, point_step=20, row_step=40, height=1, width=2, data=bytes(40))
    assert validate_pointcloud2(valid) == []
    invalid = SimpleNamespace(
        fields=fields[:-1], point_step=20, row_step=40, height=1, width=2, data=bytes(30))
    errors = validate_pointcloud2(invalid)
    assert any("timestamp or time_offset" in item for item in errors)
    assert any("data length" in item for item in errors)


def test_livox_pointcloud2_timestamp_field_is_supported():
    fields = [SimpleNamespace(name=name) for name in (
        "x", "y", "z", "intensity", "tag", "line", "timestamp")]
    message = SimpleNamespace(
        fields=fields, point_step=32, row_step=64, height=1, width=2, data=bytes(64))
    assert validate_pointcloud2(message) == []


def test_eddy_current_array_and_metadata_validation():
    valid = SimpleNamespace(
        channel_count=2,
        sample_count=3,
        signal_i=[1.0] * 6,
        signal_q=[float("nan")] * 6,
        sampling_rate_hz=0.0,
        excitation_frequency_hz=0.0,
        quality_score=0.0,
    )
    assert validate_eddy_current(valid) == []

    invalid = SimpleNamespace(**vars(valid))
    invalid.signal_q = [0.0] * 5
    invalid.quality_score = 1.1
    errors = validate_eddy_current(invalid)
    assert any("signal_q length" in item for item in errors)
    assert any("quality_score" in item for item in errors)
