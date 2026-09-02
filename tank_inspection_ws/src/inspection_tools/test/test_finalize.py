from pathlib import Path

import yaml

from inspection_tools.finalize_bag import build_manifest, compute_sha256


def test_manifest_and_sha256(tmp_path):
    mcap = tmp_path / "segment_0.mcap"
    mcap.write_bytes(b"deterministic mcap test payload")
    metadata = {
        "storage_identifier": "mcap",
        "starting_time": {"nanoseconds_since_epoch": 1_700_000_000_000_000_000},
        "duration": {"nanoseconds": 5_000_000_000},
        "message_count": 123,
        "topics_with_message_count": [{"topic_metadata": {"name": "/test"}}],
    }
    first_hash = compute_sha256(mcap)
    second_hash = compute_sha256(mcap)
    assert first_hash == second_hash
    assert len(first_hash) == 64
    manifest = build_manifest(
        tmp_path, metadata, [mcap], task_id="task-1", asset_id="tank-1",
        course_id="course-1", plate_id="plate-1")
    assert manifest["status_history"] == ["CLOSED", "HASHED"]
    assert manifest["files"][0]["sha256"] == first_hash
    serialized = yaml.safe_dump(manifest)
    assert yaml.safe_load(serialized)["message_count"] == 123
