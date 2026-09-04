"""Read a closed rosbag2 MCAP directory and emit a machine-readable integrity report."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics
import sys

import yaml

from inspection_tools.validators import (
    validate_eddy_current,
    validate_image,
    validate_paut_frame,
    validate_pointcloud2,
    validate_ultrasound,
)


IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw"
COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw"
LIDAR_TOPIC = "/livox/lidar"
ULTRASOUND_TOPIC = "/inspection/ultrasound/raw"
EDDY_CURRENT_TOPIC = "/inspection/eddy_current/raw"
PAUT_TOPIC = "/inspection/paut/raw"
FUSION_TOPIC = "/derived/inspection/fusion_index"
REQUIRED_TOPICS = {
    "/camera/camera/depth/image_rect_raw",
    "/camera/camera/depth/camera_info",
    "/camera/camera/color/image_raw",
    "/camera/camera/color/camera_info",
    "/livox/lidar",
    "/livox/imu",
    "/robot/odometry",
    "/robot/state",
    "/inspection/ultrasound/raw",
    "/inspection/eddy_current/raw",
    "/inspection/probe_state",
    "/tf",
    "/tf_static",
    "/diagnostics",
    FUSION_TOPIC,
}
EDDY_DEMO_REQUIRED_TOPICS = {
    "/camera/camera/depth/image_rect_raw",
    "/camera/camera/depth/camera_info",
    "/camera/camera/color/image_raw",
    "/camera/camera/color/camera_info",
    "/livox/lidar",
    "/livox/imu_raw",
    "/livox/imu",
    "/inspection/eddy_current/raw",
    "/tf_static",
    "/diagnostics",
    FUSION_TOPIC,
}
CACHE_LIMITS = {
    "camera": 100,
    "lidar": 40,
    "imu": 1500,
    "odometry": 500,
    "ultrasound": 200,
    "eddy_current": 200,
    "robot_state": 500,
    "probe_state": 500,
}


def _stamp_ns(message, fallback_ns):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return int(fallback_ns)
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(fallback_ns)


def _uint8_value(value):
    """Normalize ROS uint8 fields represented as either int or one-byte bytes."""
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    return int(value)


def _metadata(bag_dir):
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"missing {metadata_path}")
    document = yaml.safe_load(metadata_path.read_text())
    return document["rosbag2_bagfile_information"]


def validate(
    bag_dir, require_all_topics=True, require_calibrated_eddy=True,
    required_topics=None,
):
    # Imports are delayed so --help and unit tests remain usable outside a sourced ROS shell.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_dir).resolve()
    metadata = _metadata(bag_dir)
    errors = []
    warnings = []
    storage_identifier = metadata.get("storage_identifier")
    if storage_identifier != "mcap":
        errors.append(f"storage_identifier is {storage_identifier!r}, expected 'mcap'")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topic_stats = {
        name: {
            "type": type_name,
            "message_count": 0,
            "start_stamp_ns": None,
            "end_stamp_ns": None,
            "non_monotonic_count": 0,
        }
        for name, type_name in topic_types.items()
    }
    if require_all_topics:
        expected_topics = REQUIRED_TOPICS if required_topics is None else set(required_topics)
        for missing_topic in sorted(expected_topics - topic_types.keys()):
            errors.append(f"required topic is absent: {missing_topic}")
    message_classes = {}
    previous_stamp = {}
    fusion_total = 0
    fusion_valid = 0
    temporal_valid = 0
    temporal_steady_total = 0
    temporal_steady_valid = 0
    temporal_steady_started = False
    camera_errors = []
    lidar_errors = []
    modality_valid = Counter()
    invalid_reasons = Counter()
    diagnostic_cache_max = Counter()
    diagnostic_last = {}
    diagnostic_last_by_status = {}
    eddy_acceptance = {
        "seen": False,
        "assigned_sensor_id": False,
        "assigned_calibration_id": False,
        "positive_sampling_rate": False,
        "positive_excitation_frequency": False,
        "finite_gain": False,
        "finite_lift_off": False,
        "finite_i": False,
        "finite_q": False,
    }

    while reader.has_next():
        topic, serialized, bag_stamp_ns = reader.read_next()
        stats = topic_stats[topic]
        stats["message_count"] += 1
        try:
            message_class = message_classes.setdefault(topic, get_message(topic_types[topic]))
            message = deserialize_message(serialized, message_class)
        except Exception as exc:  # keep the report useful when a type support library is absent
            errors.append(f"{topic}: deserialization failed: {exc}")
            continue
        stamp_ns = _stamp_ns(message, bag_stamp_ns)
        stats["start_stamp_ns"] = stamp_ns if stats["start_stamp_ns"] is None else min(
            stats["start_stamp_ns"], stamp_ns)
        stats["end_stamp_ns"] = stamp_ns if stats["end_stamp_ns"] is None else max(
            stats["end_stamp_ns"], stamp_ns)
        if topic in previous_stamp and stamp_ns < previous_stamp[topic]:
            stats["non_monotonic_count"] += 1
        previous_stamp[topic] = stamp_ns

        structural_errors = []
        if topic == IMAGE_TOPIC:
            structural_errors = validate_image(
                message, expected_encodings={"16UC1", "mono16"})
        elif topic == COLOR_IMAGE_TOPIC:
            structural_errors = validate_image(
                message, expected_encodings={"rgb8", "bgr8", "rgba8", "bgra8"})
        elif topic == LIDAR_TOPIC:
            structural_errors = validate_pointcloud2(message)
        elif topic == ULTRASOUND_TOPIC:
            structural_errors = validate_ultrasound(message)
        elif topic == EDDY_CURRENT_TOPIC:
            structural_errors = validate_eddy_current(message)
            eddy_acceptance["seen"] = True
            eddy_acceptance["assigned_sensor_id"] |= bool(
                message.sensor_id and message.sensor_id != "UNASSIGNED")
            eddy_acceptance["assigned_calibration_id"] |= bool(
                message.calibration_id and message.calibration_id != "UNASSIGNED")
            eddy_acceptance["positive_sampling_rate"] |= bool(
                math.isfinite(message.sampling_rate_hz) and message.sampling_rate_hz > 0.0)
            frequency_hz = message.excitation_frequency_hz
            eddy_acceptance["positive_excitation_frequency"] |= bool(
                math.isfinite(frequency_hz) and frequency_hz > 0.0)
            eddy_acceptance["finite_gain"] |= math.isfinite(message.gain_db)
            eddy_acceptance["finite_lift_off"] |= math.isfinite(message.lift_off_mm)
            eddy_acceptance["finite_i"] |= any(math.isfinite(value) for value in message.signal_i)
            eddy_acceptance["finite_q"] |= any(math.isfinite(value) for value in message.signal_q)
        elif topic == PAUT_TOPIC:
            structural_errors = validate_paut_frame(message)
        for item in structural_errors:
            errors.append(f"{topic} message {stats['message_count']}: {item}")

        if topic == FUSION_TOPIC:
            fusion_total += 1
            all_valid = all((
                message.camera_valid,
                message.lidar_valid,
                message.imu_valid,
                message.pose_valid,
            ))
            if all_valid:
                fusion_valid += 1
            time_modalities_valid = all((
                message.camera_valid,
                message.lidar_valid,
                message.imu_valid,
            ))
            if time_modalities_valid:
                temporal_valid += 1
                temporal_steady_started = True
            if temporal_steady_started:
                temporal_steady_total += 1
                if time_modalities_valid:
                    temporal_steady_valid += 1
            for modality in ("camera", "lidar", "imu", "pose"):
                if getattr(message, modality + "_valid"):
                    modality_valid[modality] += 1
            if message.invalid_reason:
                invalid_reasons[message.invalid_reason] += 1
            if message.camera_valid:
                camera_errors.append(int(message.camera_time_error_ns))
            if message.lidar_valid:
                lidar_errors.append(int(message.lidar_time_error_ns))
        elif topic == "/diagnostics":
            diagnostic_last = {
                value.key: value.value
                for status in message.status
                for value in status.values
            }
            for status in message.status:
                diagnostic_last_by_status[status.name] = {
                    "level": _uint8_value(status.level),
                    "message": status.message,
                    "hardware_id": status.hardware_id,
                    "values": {value.key: value.value for value in status.values},
                }
            for key, value in diagnostic_last.items():
                if key.endswith(".length"):
                    try:
                        diagnostic_cache_max[key.removesuffix(".length")] = max(
                            diagnostic_cache_max[key.removesuffix(".length")], int(value))
                    except ValueError:
                        errors.append(f"diagnostic {key} is not an integer: {value!r}")

    for topic, stats in topic_stats.items():
        if stats["non_monotonic_count"]:
            errors.append(
                f"{topic}: {stats['non_monotonic_count']} header timestamp regressions detected")
        duration_ns = (stats["end_stamp_ns"] or 0) - (stats["start_stamp_ns"] or 0)
        stats["average_rate_hz"] = (
            (stats["message_count"] - 1) * 1_000_000_000 / duration_ns
            if stats["message_count"] > 1 and duration_ns > 0 else None
        )

    for cache, limit in CACHE_LIMITS.items():
        observed = diagnostic_cache_max.get(cache, 0)
        if observed > limit:
            errors.append(f"cache {cache} length {observed} exceeded configured max_count {limit}")

    if eddy_acceptance["seen"]:
        acceptance_messages = {
            "assigned_sensor_id": "sensor_id is never assigned",
            "assigned_calibration_id": "calibration_id is never assigned",
            "positive_sampling_rate": "sampling_rate_hz is never positive",
            "positive_excitation_frequency": "excitation_frequency_hz is never positive",
            "finite_gain": "gain_db is never finite",
            "finite_lift_off": "lift_off_mm is never finite",
            "finite_i": "signal_i contains no finite sample",
            "finite_q": "signal_q contains no finite sample",
        }
        acceptance_output = errors if require_calibrated_eddy else warnings
        acceptance_prefix = (
            "eddy-current acceptance blocker" if require_calibrated_eddy
            else "provisional eddy-current limitation"
        )
        for key, message in acceptance_messages.items():
            if not eddy_acceptance[key]:
                acceptance_output.append(f"{acceptance_prefix}: {message}")

    def error_summary(values):
        absolute = sorted(abs(value) for value in values)
        percentile_index = max(0, math.ceil(0.95 * len(absolute)) - 1)
        return {
            "sample_count": len(values),
            "mean_signed_ns": statistics.fmean(values) if values else None,
            "median_signed_ns": statistics.median(values) if values else None,
            "min_signed_ns": min(values) if values else None,
            "max_signed_ns": max(values) if values else None,
            "mean_abs_ns": statistics.fmean(absolute) if absolute else None,
            "p95_abs_ns": absolute[percentile_index] if absolute else None,
            "max_abs_ns": absolute[-1] if absolute else None,
        }

    report = {
        "bag_directory": str(bag_dir),
        "storage_identifier": storage_identifier,
        "relative_files": metadata.get("relative_file_paths", []),
        "metadata_message_count": metadata.get("message_count"),
        "topics": topic_stats,
        "fusion": {
            "message_count": fusion_total,
            "fully_valid_count": fusion_valid,
            "fully_valid_rate": fusion_valid / fusion_total if fusion_total else 0.0,
            "temporal_valid_count": temporal_valid,
            "temporal_valid_rate": temporal_valid / fusion_total if fusion_total else 0.0,
            "steady_state_after_first_temporal_valid": {
                "message_count": temporal_steady_total,
                "valid_count": temporal_steady_valid,
                "valid_rate": (
                    temporal_steady_valid / temporal_steady_total
                    if temporal_steady_total else 0.0
                ),
            },
            "modality_valid_count": dict(modality_valid),
            "modality_valid_rate": {
                name: modality_valid[name] / fusion_total if fusion_total else 0.0
                for name in ("camera", "lidar", "imu", "pose")
            },
            "invalid_reason_counts": dict(invalid_reasons),
            "camera_time_error": error_summary(camera_errors),
            "lidar_time_error": error_summary(lidar_errors),
        },
        "diagnostics": {
            "maximum_cache_lengths": dict(diagnostic_cache_max),
            "configured_max_counts": CACHE_LIMITS,
            "last_values": diagnostic_last,
            "last_by_status": diagnostic_last_by_status,
        },
        "errors": errors,
        "warnings": warnings,
        "valid": not errors,
    }
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_directory", type=Path)
    parser.add_argument("--output", type=Path, help="default: BAG/validation_report.json")
    parser.add_argument(
        "--allow-partial", action="store_true",
        help="validate only topics present (useful for a derived-only replay verification bag)",
    )
    parser.add_argument(
        "--allow-provisional-eddy", action="store_true",
        help=(
            "report unassigned/unknown Eddy calibration semantics as warnings; "
            "structural and timestamp errors still fail validation"
        ),
    )
    parser.add_argument(
        "--eddy-demo-profile", action="store_true",
        help=(
            "require the connected D405 + MID-360 + IMU + Eddy + FusionIndex demo topics; "
            "do not require robot, ultrasound or odometry topics"
        ),
    )
    args = parser.parse_args(argv)
    output_path = args.output or args.bag_directory / "validation_report.json"
    try:
        report = validate(
            args.bag_directory,
            require_all_topics=not args.allow_partial,
            require_calibrated_eddy=not args.allow_provisional_eddy,
            required_topics=(
                EDDY_DEMO_REQUIRED_TOPICS if args.eddy_demo_profile else None),
        )
    except Exception as exc:
        report = {
            "bag_directory": str(args.bag_directory.resolve()),
            "valid": False,
            "errors": [f"validation could not run: {exc}"],
            "warnings": [],
        }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"validation_report={output_path.resolve()}")
    print(f"valid={str(report['valid']).lower()} errors={len(report['errors'])}")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
