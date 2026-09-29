"""Small reusable structural validators shared by the bag reader and tests."""

import math


def validate_image(message, expected_encodings=None):
    errors = []
    bytes_per_pixel = {
        "mono8": 1,
        "8UC1": 1,
        "mono16": 2,
        "16UC1": 2,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }
    if expected_encodings is not None and message.encoding not in expected_encodings:
        errors.append(
            f"image encoding {message.encoding!r} is not one of "
            f"{sorted(expected_encodings)}")
    pixel_size = bytes_per_pixel.get(message.encoding)
    if pixel_size is None:
        errors.append(f"unsupported image encoding for structural validation: {message.encoding!r}")
        pixel_size = 1
    minimum_step = int(message.width) * pixel_size
    if message.height <= 0 or message.width <= 0:
        errors.append("image dimensions must be positive")
    if message.step < minimum_step:
        errors.append(f"image step {message.step} is less than minimum {minimum_step}")
    expected = int(message.height) * int(message.step)
    if len(message.data) != expected:
        errors.append(f"image data length {len(message.data)} != height*step {expected}")
    return errors


def validate_pointcloud2(message):
    errors = []
    field_names = [field.name for field in message.fields]
    fields = set(field_names)
    required = {"x", "y", "z", "intensity"}
    missing = sorted(required - fields)
    if missing:
        errors.append("PointCloud2 missing fields: " + ", ".join(missing))
    if "timestamp" not in fields and "time_offset" not in fields:
        errors.append("PointCloud2 missing per-point time field: timestamp or time_offset")
    if len(field_names) != len(fields):
        errors.append("PointCloud2 field names must be unique")
    if message.point_step <= 0:
        errors.append("PointCloud2 point_step must be positive")
    if message.height <= 0 or message.width <= 0:
        errors.append("PointCloud2 height and width must be positive")
    minimum_row_step = int(message.width) * int(message.point_step)
    if message.row_step < minimum_row_step:
        errors.append(
            f"PointCloud2 row_step {message.row_step} < width*point_step {minimum_row_step}")
    expected_data = int(message.row_step) * int(message.height)
    if len(message.data) != expected_data:
        errors.append(f"PointCloud2 data length {len(message.data)} != {expected_data}")

    datatype_sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    for field in message.fields:
        datatype = getattr(field, "datatype", None)
        offset = getattr(field, "offset", None)
        count = getattr(field, "count", 1)
        if datatype is None or offset is None:
            continue
        if datatype not in datatype_sizes or count <= 0:
            errors.append(f"PointCloud2 field {field.name!r} has invalid datatype/count")
            continue
        field_end = int(offset) + datatype_sizes[datatype] * int(count)
        if offset < 0 or field_end > message.point_step:
            errors.append(
                f"PointCloud2 field {field.name!r} exceeds point_step {message.point_step}")
    return errors


def validate_ultrasound(message):
    expected = int(message.channel_count) * int(message.sample_count)
    if len(message.samples) != expected:
        return [
            f"Ultrasound samples length {len(message.samples)} != "
            f"channel_count*sample_count {expected}"
        ]
    return []


def validate_eddy_current(message):
    errors = []
    channel_count = int(message.channel_count)
    sample_count = int(message.sample_count)
    if channel_count <= 0 or sample_count <= 0:
        errors.append("EddyCurrentFrame channel_count and sample_count must be positive")
    expected = channel_count * sample_count
    if len(message.signal_i) != expected:
        errors.append(
            f"EddyCurrentFrame signal_i length {len(message.signal_i)} != "
            f"channel_count*sample_count {expected}")
    if len(message.signal_q) != expected:
        errors.append(
            f"EddyCurrentFrame signal_q length {len(message.signal_q)} != "
            f"channel_count*sample_count {expected}")
    if not math.isfinite(message.sampling_rate_hz) or message.sampling_rate_hz < 0.0:
        errors.append("EddyCurrentFrame sampling_rate_hz must be finite and non-negative")
    if (not math.isfinite(message.excitation_frequency_hz)
            or message.excitation_frequency_hz < 0.0):
        errors.append(
            "EddyCurrentFrame excitation_frequency_hz must be finite and non-negative")
    if not math.isfinite(message.quality_score) or not 0.0 <= message.quality_score <= 1.0:
        errors.append("EddyCurrentFrame quality_score must be finite and in [0, 1]")
    return errors


def validate_paut_frame(message):
    errors = []
    channel_count = int(message.channel_count)
    sample_count = int(message.sample_count)
    if channel_count <= 0 or sample_count <= 0:
        errors.append("PautFrame channel_count and sample_count must be positive")
    expected = channel_count * sample_count
    if len(message.samples) != expected:
        errors.append(
            f"PautFrame samples length {len(message.samples)} != "
            f"channel_count*sample_count {expected}")
    return errors


# ---------------------------------------------------------------------------
# PAUT v1（:12346 → PautFrameV2 + PautConfig）
#
# 与旧 PautFrame 的区别：v1 带完整 61x896 uint8 原始数据 + 40 字段采集配置，
# 两者靠 config_seq 配对（契约 C7.2）。该配对是**跨消息**检查，放在 validate_bag.py。
# ---------------------------------------------------------------------------

# status_flags 位（与 PautFrameV2.msg 的常量一致，位号勿改）
PAUT_STATUS_HAS_DATA = 1
PAUT_STATUS_SATURATED = 2
PAUT_STATUS_ENCODER_VALID = 8
PAUT_STATUS_CALIBRATED = 32
PAUT_STATUS_SAMPLE_DTYPE_U8 = 128

# 枚举取值上界（与消息内 const 一致）
_PAUT_TS_MAX = 3          # TS_UNKNOWN..TS_HOST_RECEIVE
_PAUT_RATE_SRC_MAX = 1    # UNKNOWN, DERIVED_UNVERIFIED
_PAUT_ENC_UNIT_MAX = 2    # UNKNOWN, COUNT_PER_MM, MM_PER_COUNT
_PAUT_SAMPLE_DTYPE_MAX = 4
_PAUT_SCAN_MODE_MAX = 4   # LINEAR, SECTOR, TOFM, TOFD, PWI
_PAUT_BOARD_TYPE_MAX = 8
_PAUT_SCHEMA_VERSION = 1


def _paut_unknown_or_nonneg(value):
    """契约 C4: 浮点「未知」用 NaN。故 NaN 合法，负数非法。"""
    return math.isnan(value) or (math.isfinite(value) and value >= 0.0)


def validate_paut_config(message):
    """PautConfig 结构校验。只查**消息自身自洽性**，不查跨消息配对。"""
    errors = []

    if int(message.config_seq) <= 0:
        errors.append("PautConfig config_seq must be positive")

    beam_count = int(message.beam_count)
    element_total = int(message.element_total)
    if beam_count <= 0:
        errors.append("PautConfig beam_count must be positive")
    if element_total <= 0:
        errors.append("PautConfig element_total must be positive")
    if beam_count > element_total:
        # 波束数由阵元数算出（(阵元-孔径)/步进+1），不可能多于阵元数
        errors.append(
            f"PautConfig beam_count {beam_count} > element_total {element_total}")

    if not math.isfinite(message.element_pitch_mm) or message.element_pitch_mm <= 0.0:
        errors.append("PautConfig element_pitch_mm must be finite and positive")

    if int(message.sample_point_num) <= 0:
        errors.append("PautConfig sample_point_num must be positive")

    # 采样率：设备不直接提供，由 sample_point_num/range_ns 反推。
    # range_ns 为 0 时反推不出，允许 NaN（契约 C4「未知」）。
    if not _paut_unknown_or_nonneg(message.sampling_rate_hz):
        errors.append("PautConfig sampling_rate_hz must be NaN (unknown) or non-negative")

    if int(message.sampling_rate_src) > _PAUT_RATE_SRC_MAX:
        errors.append(
            f"PautConfig sampling_rate_src {int(message.sampling_rate_src)} out of range")
    if int(message.sample_dtype) > _PAUT_SAMPLE_DTYPE_MAX:
        errors.append(
            f"PautConfig sample_dtype {int(message.sample_dtype)} out of range")
    if int(message.scan_mode) > _PAUT_SCAN_MODE_MAX:
        errors.append(f"PautConfig scan_mode {int(message.scan_mode)} out of range")
    if int(message.board_type) > _PAUT_BOARD_TYPE_MAX:
        errors.append(f"PautConfig board_type {int(message.board_type)} out of range")
    if int(message.enc1_unit) > _PAUT_ENC_UNIT_MAX:
        errors.append(f"PautConfig enc1_unit {int(message.enc1_unit)} out of range")
    if int(message.enc2_unit) > _PAUT_ENC_UNIT_MAX:
        errors.append(f"PautConfig enc2_unit {int(message.enc2_unit)} out of range")

    return errors


def validate_paut_frame_v2(message):
    """PautFrameV2 结构校验（契约 C7.1 维度自洽 / C7.3 状态位一致）。"""
    errors = []

    if int(message.schema_version) != _PAUT_SCHEMA_VERSION:
        errors.append(
            f"PautFrameV2 schema_version {int(message.schema_version)} != "
            f"{_PAUT_SCHEMA_VERSION}")

    beam_count = int(message.beam_count)
    sample_count = int(message.sample_count)
    if beam_count <= 0 or sample_count <= 0:
        errors.append("PautFrameV2 beam_count and sample_count must be positive")

    expected = beam_count * sample_count
    if len(message.samples) != expected:
        errors.append(
            f"PautFrameV2 samples length {len(message.samples)} != "
            f"beam_count*sample_count {expected}")

    if int(message.timestamp_source) > _PAUT_TS_MAX:
        errors.append(
            f"PautFrameV2 timestamp_source {int(message.timestamp_source)} out of range")

    flags = int(message.status_flags)
    if not flags & PAUT_STATUS_HAS_DATA:
        errors.append("PautFrameV2 status_flags missing STATUS_HAS_DATA")

    # C7.3 状态位与字段值一致
    saturated_count = int(message.saturated_count)
    if flags & PAUT_STATUS_SATURATED and saturated_count == 0:
        errors.append("PautFrameV2 STATUS_SATURATED set but saturated_count == 0")
    if saturated_count > expected:
        errors.append(
            f"PautFrameV2 saturated_count {saturated_count} > total samples {expected}")

    return errors
