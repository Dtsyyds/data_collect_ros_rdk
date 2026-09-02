"""Estimate uncompressed acquisition bandwidth and capacity from sensor settings."""

import argparse
import json


def estimate(args):
    camera_bps = args.camera_width * args.camera_height * args.camera_bytes_per_pixel * args.camera_fps
    lidar_bps = args.lidar_points_per_frame * args.lidar_bytes_per_point * args.lidar_rate_hz
    ultrasound_bps = (
        args.ultrasound_channels * args.ultrasound_samples *
        args.ultrasound_bytes_per_sample * args.ultrasound_rate_hz
    )
    total_bps = camera_bps + lidar_bps + ultrasound_bps
    mb_s = total_bps / 1_000_000
    gb_h = total_bps * 3600 / 1_000_000_000
    return {
        "camera_MB_s": camera_bps / 1_000_000,
        "lidar_MB_s": lidar_bps / 1_000_000,
        "ultrasound_MB_s": ultrasound_bps / 1_000_000,
        "total_MB_s": mb_s,
        "total_GB_h": gb_h,
        "task_duration_minutes": args.task_duration_minutes,
        "task_total_GB": gb_h * args.task_duration_minutes / 60,
        "note": "Uncompressed payload estimate; protocol overhead and MCAP compression vary.",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-width", type=int, default=848)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=15.0)
    parser.add_argument("--camera-bytes-per-pixel", type=int, default=2)
    parser.add_argument("--lidar-points-per-frame", type=int, default=2400)
    parser.add_argument("--lidar-rate-hz", type=float, default=10.0)
    parser.add_argument("--lidar-bytes-per-point", type=int, default=20)
    parser.add_argument("--ultrasound-channels", type=int, default=2)
    parser.add_argument("--ultrasound-samples", type=int, default=2048)
    parser.add_argument("--ultrasound-rate-hz", type=float, default=50.0)
    parser.add_argument("--ultrasound-bytes-per-sample", type=int, default=2)
    parser.add_argument("--task-duration-minutes", type=float, default=60.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if any(value <= 0 for value in vars(args).values() if isinstance(value, (int, float))):
        parser.error("all numeric inputs must be positive")
    result = estimate(args)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f"Total: {result['total_MB_s']:.3f} MB/s")
        print(f"Hourly: {result['total_GB_h']:.3f} GB/h")
        print(f"Task: {result['task_total_GB']:.3f} GB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
