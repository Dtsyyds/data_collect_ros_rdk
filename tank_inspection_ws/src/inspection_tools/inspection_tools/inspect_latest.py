"""Validate and summarize the newest closed Eddy synchronization demo bag."""

import argparse
import json
from pathlib import Path
import sys

from inspection_tools.validate_bag import EDDY_DEMO_REQUIRED_TOPICS, validate
import yaml


DEMO_TOPICS = (
    ("Eddy raw", "/inspection/eddy_current/raw"),
    ("FusionIndex", "/derived/inspection/fusion_index"),
    ("D405 depth", "/camera/camera/depth/image_rect_raw"),
    ("D405 color", "/camera/camera/color/image_raw"),
    ("MID-360 cloud", "/livox/lidar"),
    ("MID-360 IMU", "/livox/imu"),
)


def latest_closed_bag(root: Path) -> Path:
    root = Path(root).expanduser().resolve()
    candidates = [
        path for path in root.glob("eddy_sync_loop_*")
        if path.is_dir() and (path / "metadata.yaml").is_file()
    ]
    if not candidates:
        raise ValueError(f"no closed eddy_sync_loop_* bag found below {root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _resolve_bag(value, bag_root):
    if value is None:
        return latest_closed_bag(bag_root)
    bag = Path(value).expanduser().resolve()
    if not (bag / "metadata.yaml").is_file():
        raise ValueError(f"not a closed rosbag2 directory: {bag}")
    return bag


def _metadata(bag):
    document = yaml.safe_load((bag / "metadata.yaml").read_text())
    return document["rosbag2_bagfile_information"]


def _rate_text(topic):
    rate = topic.get("average_rate_hz")
    return "n/a" if rate is None else f"{rate:.3f} Hz"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag_directory", nargs="?", type=Path,
        help="closed bag directory; omitted selects the newest bags/eddy_sync_loop_* bag",
    )
    parser.add_argument("--bag-root", type=Path, default=Path.cwd() / "bags")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    try:
        bag = _resolve_bag(args.bag_directory, args.bag_root)
        metadata = _metadata(bag)
        report = validate(
            bag,
            require_all_topics=True,
            require_calibrated_eddy=False,
            required_topics=EDDY_DEMO_REQUIRED_TOPICS,
        )
    except Exception as exc:
        print(f"DEMO CHECK FAIL: {exc}", file=sys.stderr)
        return 1

    output = args.output or bag / "validation_report.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    duration_s = int(metadata.get("duration", {}).get("nanoseconds", 0)) / 1.0e9
    total_size = sum(
        (bag / relative).stat().st_size
        for relative in metadata.get("relative_file_paths", [])
        if (bag / relative).is_file()
    )
    print("\n=== Tank Inspection Demo Bag ===")
    print(f"bag: {bag}")
    print(f"duration: {duration_s:.3f} s")
    print(f"MCAP size: {total_size / (1024 ** 2):.1f} MiB")
    print(f"messages: {metadata.get('message_count', 0)}")
    print("\nTopic summary:")
    for label, name in DEMO_TOPICS:
        topic = report.get("topics", {}).get(name)
        if topic is None:
            print(f"  {label:16s} MISSING")
        else:
            print(
                f"  {label:16s} {topic['message_count']:7d}  {_rate_text(topic)}")

    fusion = report.get("fusion", {})
    steady = fusion.get("steady_state_after_first_temporal_valid", {})
    diagnostics = report.get("diagnostics", {}).get("last_by_status", {})
    sync_values = diagnostics.get(
        "inspection_sync/bounded_caches", {}).get("values", {})
    eddy_values = diagnostics.get(
        "eddy_driver/acquisition", {}).get("values", {})
    print("\nSoftware-time matching:")
    print(
        "  steady temporal valid: "
        f"{steady.get('valid_count', 0)}/{steady.get('message_count', 0)} "
        f"({100.0 * steady.get('valid_rate', 0.0):.3f}%)")
    print(f"  sync pending_dropped: {sync_values.get('pending_dropped', 'n/a')}")
    print(f"  Eddy dropped frames: {eddy_values.get('dropped_frame_count', 'n/a')}")
    print(f"\nvalidation report: {output.resolve()}")
    print(
        f"result: {'PASS' if report['valid'] else 'FAIL'}; "
        f"errors={len(report['errors'])}, provisional_warnings={len(report['warnings'])}")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
