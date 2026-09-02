import numpy as np

import inspection_rerun.rerun_session as module


def test_set_time_passes_explicit_nanosecond_datetime(monkeypatch):
    calls = []
    monkeypatch.setattr(module.rr, "set_time", lambda timeline, **kwargs: calls.append((timeline, kwargs)))
    session = object.__new__(module.RerunSession)
    session.set_time(1_787_556_982_001_256_175, 9)
    assert calls[0][0] == "ros_time"
    assert calls[0][1]["timestamp"] == np.datetime64(1_787_556_982_001_256_175, "ns")
    assert calls[1] == ("frame_sequence", {"sequence": 9})


def test_log_accepts_linked_2d_and_3d_entities(monkeypatch):
    calls = []
    monkeypatch.setattr(module.rr, "log", lambda *args, **kwargs: calls.append((args, kwargs)))
    session = object.__new__(module.RerunSession)
    session.log("world/linked", "three-d", "two-d", static=True)
    assert calls == [(('world/linked', 'three-d', 'two-d'), {'static': True})]


def test_coordinate_frame_uses_ros_frame_id(monkeypatch):
    calls = []
    monkeypatch.setattr(module.rr, "CoordinateFrame", lambda frame: calls.append(frame) or frame)
    assert module.coordinate_frame_entity("/livox_frame") == "livox_frame"
    assert calls == ["livox_frame"]


def test_pinhole_uses_optical_frame_as_parent(monkeypatch):
    calls = []
    monkeypatch.setattr(module.rr, "Pinhole", lambda **kwargs: calls.append(kwargs) or kwargs)
    entity = module.pinhole_entity([10.0, 10.0], [5.0, 5.0], [10, 10],
                                   "camera_depth_optical_frame")
    assert entity["parent_frame"] == "camera_depth_optical_frame"
    assert "child_frame" not in entity
