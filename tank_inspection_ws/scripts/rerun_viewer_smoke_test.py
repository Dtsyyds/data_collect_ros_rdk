#!/usr/bin/env python3
"""Small Rerun smoke test; save mode is intentionally the default."""

import argparse
from pathlib import Path

import numpy as np
import rerun as rr


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-mode", choices=("save", "spawn", "connect"), default="save")
    parser.add_argument("--save-path", default="output/rerun/smoke_test.rrd")
    parser.add_argument("--connect-url", default="rerun+http://127.0.0.1:9876/proxy")
    args = parser.parse_args()

    rr.init("tank_inspection_rerun_smoke", recording_id="smoke_test")
    if args.output_mode == "save":
        path = Path(args.save_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        rr.save(path)
    elif args.output_mode == "spawn":
        rr.spawn()
    else:
        rr.connect_grpc(args.connect_url)

    image = np.zeros((48, 64), dtype=np.uint8)
    image[12:36, 16:48] = 220
    points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.2], [0.0, 1.0, 0.4]], dtype=np.float32)
    for sequence in range(3):
        rr.set_time("frame_sequence", sequence=sequence)
        time_ns = 1_000_000_000 + sequence * 100_000_000
        rr.set_time("ros_time", timestamp=np.datetime64(time_ns, "ns"))
        rr.log("world/smoke_points", rr.Points3D(points + [0.0, 0.0, sequence * 0.1]))
        rr.log("world/smoke_image", rr.Image(image))
        rr.log("signals/smoke", rr.Scalars([float(sequence)]))
    rr.disconnect()
    if args.output_mode == "save":
        print(Path(args.save_path).resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
