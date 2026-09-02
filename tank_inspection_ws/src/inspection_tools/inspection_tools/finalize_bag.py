"""Finalize a closed immutable MCAP bag with hashes, manifest, and verified local NAS copy."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

import yaml


def compute_sha256(path, block_size=4 * 1024 * 1024):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while block := stream.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def _read_metadata(bag_dir):
    path = Path(bag_dir) / "metadata.yaml"
    if not path.is_file():
        raise ValueError(f"closed bag metadata is missing: {path}")
    return yaml.safe_load(path.read_text())["rosbag2_bagfile_information"]


def ensure_closed(bag_dir):
    bag_dir = Path(bag_dir)
    metadata = _read_metadata(bag_dir)
    if metadata.get("storage_identifier") != "mcap":
        raise ValueError("bag storage_identifier is not mcap")
    transient = list(bag_dir.glob("*.active")) + list(bag_dir.glob("*.tmp"))
    if transient:
        raise ValueError("bag still has transient files: " + ", ".join(map(str, transient)))
    relative_files = metadata.get("relative_file_paths", [])
    if not relative_files:
        raise ValueError("metadata contains no MCAP files")
    files = []
    for relative in relative_files:
        path = bag_dir / relative
        if path.suffix != ".mcap" or not path.is_file():
            raise ValueError(f"missing MCAP referenced by metadata: {path}")
        files.append(path)
    return metadata, files


def _time_iso(nanoseconds):
    return datetime.fromtimestamp(int(nanoseconds) / 1_000_000_000, tz=timezone.utc).isoformat()


def build_manifest(bag_dir, metadata, mcap_files, *, task_id, asset_id, course_id, plate_id):
    start_ns = int(metadata["starting_time"]["nanoseconds_since_epoch"])
    duration_ns = int(metadata["duration"]["nanoseconds"])
    files = []
    for path in mcap_files:
        files.append({
            "path": path.name,
            "size_bytes": path.stat().st_size,
            "sha256": compute_sha256(path),
        })
    return {
        "schema_version": 1,
        "task_id": task_id,
        "asset_id": asset_id,
        "course_id": course_id,
        "plate_id": plate_id,
        "bag_directory": Path(bag_dir).name,
        "storage_identifier": metadata.get("storage_identifier"),
        "start_time_utc": _time_iso(start_ns),
        "end_time_utc": _time_iso(start_ns + duration_ns),
        "total_size_bytes": sum(item["size_bytes"] for item in files),
        "topic_count": len(metadata.get("topics_with_message_count", [])),
        "message_count": int(metadata.get("message_count", 0)),
        "files": files,
        "status": "HASHED",
        "status_history": ["CLOSED", "HASHED"],
        "local_original_deleted": False,
    }


def _write_manifest(path, manifest):
    Path(path).write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True))


def finalize(
    bag_dir, *, task_id, asset_id, course_id, plate_id,
    validation_report=None, remote_store=None,
):
    bag_dir = Path(bag_dir).resolve()
    metadata, mcap_files = ensure_closed(bag_dir)
    if validation_report is not None:
        validation_report = Path(validation_report).resolve()
        report = json.loads(validation_report.read_text())
        if not report.get("valid"):
            raise ValueError("validation report is not valid; refusing VERIFIED state")
    manifest = build_manifest(
        bag_dir, metadata, mcap_files, task_id=task_id, asset_id=asset_id,
        course_id=course_id, plate_id=plate_id)
    if validation_report is not None:
        report_copy = bag_dir / "validation_report.json"
        if validation_report != report_copy:
            if report_copy.exists():
                raise FileExistsError(f"validation report destination already exists: {report_copy}")
            shutil.copy2(validation_report, report_copy)
        manifest["validation_report"] = report_copy.name
    manifest_path = bag_dir / "manifest.yaml"
    _write_manifest(manifest_path, manifest)

    remote_path = None
    if remote_store is not None:
        remote_path = Path(remote_store).resolve() / task_id / bag_dir.name
        if remote_path.exists():
            raise FileExistsError(f"remote destination already exists; no files overwritten: {remote_path}")
        remote_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(bag_dir, remote_path)
        manifest["status"] = "COPIED"
        manifest["status_history"].append("COPIED")
        manifest["remote_copy"] = str(remote_path)
        for item in manifest["files"]:
            copied = remote_path / item["path"]
            copied_hash = compute_sha256(copied)
            if copied_hash != item["sha256"]:
                raise IOError(f"SHA256 mismatch after copy: {copied}")
            item["remote_sha256"] = copied_hash
        manifest["status"] = "VERIFIED"
        manifest["status_history"].append("VERIFIED")
        _write_manifest(manifest_path, manifest)
        _write_manifest(remote_path / "manifest.yaml", manifest)
    return manifest_path, remote_path, manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_directory", type=Path)
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--asset-id", required=True)
    parser.add_argument("--course-id", required=True)
    parser.add_argument("--plate-id", required=True)
    parser.add_argument("--validation-report", type=Path)
    parser.add_argument("--remote-store", type=Path)
    args = parser.parse_args(argv)
    try:
        manifest_path, remote_path, manifest = finalize(
            args.bag_directory,
            task_id=args.task_id,
            asset_id=args.asset_id,
            course_id=args.course_id,
            plate_id=args.plate_id,
            validation_report=args.validation_report,
            remote_store=args.remote_store,
        )
    except Exception as exc:
        print(f"finalize_bag error: {exc}", file=sys.stderr)
        return 1
    print(f"manifest={manifest_path}")
    print(f"status={manifest['status']}")
    if remote_path is not None:
        print(f"remote_copy={remote_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
