from types import SimpleNamespace

from inspection_tools.validate_bag import _uint8_value
from inspection_tools.validators import (
    validate_eddy_current,
    validate_image,
    validate_paut_config,
    validate_paut_frame,
    validate_paut_frame_v2,
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


def test_paut_frame_array_length_validation():
    valid = SimpleNamespace(channel_count=167, sample_count=61, samples=list(range(167 * 61)))
    assert validate_paut_frame(valid) == []

    zero_dim = SimpleNamespace(channel_count=0, sample_count=61, samples=[])
    errors = validate_paut_frame(zero_dim)
    assert any("channel_count and sample_count must be positive" in item for item in errors)

    invalid = SimpleNamespace(
        channel_count=167, sample_count=61, samples=list(range(100)))
    errors = validate_paut_frame(invalid)
    assert any("samples length" in item for item in errors)


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


# ---------------------------------------------------------------------------
# PAUT v1 (:12346 → PautFrameV2 + PautConfig)
# ---------------------------------------------------------------------------


def test_paut_config_validation():
    valid = SimpleNamespace(
        config_seq=4,
        beam_count=61,
        element_total=64,
        element_pitch_mm=0.6,
        sample_point_num=1000,
        sampling_rate_hz=1.0e7,
        sampling_rate_src=1,   # DERIVED_UNVERIFIED（设备反推值）
        sample_dtype=0,        # u8
        scan_mode=0,           # LINEAR
        board_type=3,
        enc1_unit=0,           # UNKNOWN
        enc2_unit=0,
    )
    assert validate_paut_config(valid) == []

    # 契约 C4: 采样率未知用 NaN 是合法的（range_ns 为 0 时反推不出）
    unknown_rate = SimpleNamespace(**vars(valid))
    unknown_rate.sampling_rate_hz = float("nan")
    assert validate_paut_config(unknown_rate) == []

    bad = SimpleNamespace(**vars(valid))
    bad.beam_count = 99        # 波束数 > 阵元数，不可能
    bad.element_pitch_mm = 0.0  # 必须为正
    bad.scan_mode = 9          # 枚举越界
    errors = validate_paut_config(bad)
    assert any("beam_count" in item for item in errors)
    assert any("element_pitch_mm" in item for item in errors)
    assert any("scan_mode" in item for item in errors)


def test_paut_frame_v2_validation():
    valid = SimpleNamespace(
        schema_version=1,
        beam_count=61,
        sample_count=896,
        samples=[0] * (61 * 896),
        timestamp_source=3,           # TS_HOST_RECEIVE
        status_flags=1 | 128,         # HAS_DATA | SAMPLE_DTYPE_U8
        saturated_count=0,
    )
    assert validate_paut_frame_v2(valid) == []

    bad = SimpleNamespace(**vars(valid))
    bad.samples = [0] * 100           # 长度与维度不符（契约 C7.1）
    bad.timestamp_source = 9          # 枚举越界
    bad.status_flags = 2              # 置了 SATURATED 却没置 HAS_DATA
    errors = validate_paut_frame_v2(bad)
    assert any("samples length" in item for item in errors)
    assert any("timestamp_source" in item for item in errors)
    assert any("STATUS_HAS_DATA" in item for item in errors)

    # 契约 C7.3: 状态位与字段值必须一致
    inconsistent = SimpleNamespace(**vars(valid))
    inconsistent.status_flags = 1 | 2 | 128   # 置了 SATURATED
    inconsistent.saturated_count = 0          # 却报 0
    errors = validate_paut_frame_v2(inconsistent)
    assert any("STATUS_SATURATED set but saturated_count == 0" in item for item in errors)

    # 饱和样本数不可能超过总样本数
    too_many = SimpleNamespace(**vars(valid))
    too_many.status_flags = 1 | 2 | 128
    too_many.saturated_count = 61 * 896 + 1
    errors = validate_paut_frame_v2(too_many)
    assert any("saturated_count" in item for item in errors)
