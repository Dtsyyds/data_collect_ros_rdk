"""Adapters for the exact inspection_interfaces message definitions in this workspace."""

import math
from typing import Any

import numpy as np

from .rerun_session import scalar_entity, tensor_entity, text_entity
from .timestamp_utils import stamp_to_ns


def log_eddy(session: Any, message: Any) -> None:
    channels = int(message.channel_count)
    samples = int(message.sample_count)
    if channels <= 0 or samples <= 0:
        raise ValueError("eddy channel_count and sample_count must be positive")
    expected = channels * samples
    if len(message.signal_i) != expected or len(message.signal_q) != expected:
        raise ValueError("eddy I/Q lengths must equal channel_count*sample_count")
    i_values = np.asarray(message.signal_i, dtype=np.float32).reshape(channels, samples)
    q_values = np.asarray(message.signal_q, dtype=np.float32).reshape(channels, samples)
    session.log("signals/eddy/signal_i", tensor_entity(i_values, ["channel", "sample"]))
    session.log("signals/eddy/signal_q", tensor_entity(q_values, ["channel", "sample"]))
    for channel in range(channels):
        base = "signals/eddy/channel_%d" % channel
        session.log(base + "/i_latest", scalar_entity(i_values[channel, -1]))
        session.log(base + "/q_latest", scalar_entity(q_values[channel, -1]))
        amplitude = np.sqrt(i_values[channel] ** 2 + q_values[channel] ** 2)
        session.log(base + "/amplitude_rms", scalar_entity(np.sqrt(np.mean(amplitude ** 2))))
    for field in ("sequence", "sampling_rate_hz", "excitation_frequency_hz", "gain_db",
                  "lift_off_mm", "quality_score"):
        session.log("signals/eddy/metadata/" + field, scalar_entity(getattr(message, field)))


def log_ultrasound(session: Any, message: Any) -> None:
    channels = int(message.channel_count)
    samples = int(message.sample_count)
    if channels <= 0 or samples <= 0:
        raise ValueError("ultrasound channel_count and sample_count must be positive")
    expected = channels * samples
    if len(message.samples) != expected:
        raise ValueError("ultrasound sample length is inconsistent with dimensions")
    values = np.asarray(message.samples, dtype=np.float32).reshape(
        channels, samples)
    session.log("signals/ultrasound/waveform", tensor_entity(values, ["channel", "sample"]))
    for channel in range(channels):
        session.log("signals/ultrasound/channel_%d/latest" % channel,
                    scalar_entity(values[channel, -1]))
        session.log("signals/ultrasound/channel_%d/rms" % channel,
                    scalar_entity(np.sqrt(np.mean(values[channel] ** 2))))


def log_probe(session: Any, message: Any) -> None:
    for field in ("contact_force_n", "lift_off_mm", "temperature_c", "coupling_score",
                  "quality_flags"):
        session.log("world/probe/" + field, scalar_entity(getattr(message, field)))
    session.log("world/probe/contact_valid", scalar_entity(int(message.contact_valid)))


def log_robot(session: Any, message: Any) -> None:
    for field in ("linear_speed_m_s", "angular_speed_rad_s", "battery_voltage",
                  "adsorption_current_a", "slip_ratio", "state_flags"):
        session.log("world/robot/state/" + field, scalar_entity(getattr(message, field)))
    session.log("world/robot/state/adsorption_valid", scalar_entity(int(message.adsorption_valid)))
    session.log("world/robot/state/scan_enabled", scalar_entity(int(message.scan_enabled)))


def log_fusion_index(session: Any, message: Any) -> None:
    session.log("synchronization/anchor_sequence", scalar_entity(message.anchor_sequence))
    session.log("synchronization/camera_delta", scalar_entity(message.camera_time_error_ns))
    session.log("synchronization/lidar_delta", scalar_entity(message.lidar_time_error_ns))
    for field in ("camera_valid", "lidar_valid", "imu_valid", "pose_valid",
                  "asset_coordinate_valid"):
        session.log("synchronization/status/" + field, scalar_entity(int(getattr(message, field))))
    session.log("synchronization/pose_confidence", scalar_entity(message.pose_confidence))
    session.log("synchronization/quality_flags", scalar_entity(message.quality_flags))
    for field in ("camera_stamp", "lidar_stamp", "imu_start_stamp", "imu_end_stamp",
                  "pose_before_stamp", "pose_after_stamp"):
        session.log("synchronization/stamps/" + field,
                    text_entity(str(stamp_to_ns(getattr(message, field)))))
    if message.invalid_reason:
        session.log("synchronization/invalid_reason", text_entity(message.invalid_reason, "WARN"))
    ids = "anchor_topic=%s asset=%s course=%s plate=%s weld=%s" % (
        message.anchor_topic, message.asset_id, message.course_id, message.plate_id, message.weld_id)
    session.log("synchronization/associations", text_entity(ids))
