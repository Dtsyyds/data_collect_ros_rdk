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
