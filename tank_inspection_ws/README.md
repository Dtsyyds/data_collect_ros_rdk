# Tank inspection ROS 2 acquisition prototype

中文现场演示流程见 [DEMO_WORKFLOW_ZH.md](DEMO_WORKFLOW_ZH.md)。标准 Demo 已简化为一键
采集、自动检查最新包和一键 RViz 回放三个入口。

This ROS 2 Jazzy workspace implements bounded multimodal acquisition, temporal indexing,
split MCAP storage, validation, immutable-file hashing, a verified local NAS-style copy,
and playback regeneration. It does **not** implement Physics-BEV or claim complete multimodal
hardware validation; the D405 depth/color path and MID-360 point-cloud/IMU path have real-device
smoke tests, while their calibrated multimodal fusion is still pending.

Real-device startup, validation, D405 color/depth operation, and new-device onboarding are
documented in [HARDWARE_INTEGRATION_GUIDE.md](HARDWARE_INTEGRATION_GUIDE.md).
RK3588 arm64 system preparation, ROS/MCAP/RealSense/Livox installation, native build, and
board-level acceptance are documented in [RK3588_PORTING_GUIDE.md](RK3588_PORTING_GUIDE.md).

## Workspace layout

```text
tank_inspection_ws/
├── src/
│   ├── inspection_interfaces/     # Raw sensor and FusionIndex messages
│   ├── inspection_sync/           # C++ bounded caches, worker, matching, SLERP, diagnostics
│   ├── inspection_mock_sensors/   # Deterministic configurable local sensor publishers
│   ├── inspection_bringup/        # Launch files, MCAP, QoS, cache and mock configuration
│   ├── inspection_tools/          # Validation, manifest/hash/copy and capacity tools
│   ├── inspection_livox_adapter/  # MID-360 raw-g IMU to ROS SI-unit adapter
│   ├── eddy_driver/               # STM32 USB-UART eddy acquisition and canonical adapter
│   └── livox_ros_driver2/         # Local Livox ROS Driver 2 source (PointCloud2 mode)
├── bags/                          # Closed local rosbag2/MCAP data
├── remote_store/                  # Verified local NAS simulation (never public cloud)
└── reports/                       # Validation reports and ROS logs
```

## Clean ROS shell

The development host had Conda `base` and several unrelated overlays active. Run ROS commands
from the workspace root with a clean shell that uses system Python 3.12:

```bash
env -i USER="$USER" LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin ROS_LOG_DIR="$PWD/reports/ros_logs" \
  /usr/bin/bash --noprofile --norc
source /opt/ros/jazzy/setup.bash
```

## Verified commands

Build:

```bash
colcon build --symlink-install
source install/setup.bash
```

Run mock sensors and bounded synchronization without recording:

```bash
ros2 launch inspection_bringup mock_pipeline.launch.py record:=false
```

Run the complete mock chain with debug-size MCAP splitting (choose a new output path):

```bash
ros2 launch inspection_bringup mock_pipeline.launch.py \
  record:=true debug_mode:=true output_directory:="$PWD/bags/mock_run"
```

Record only already-running hardware or mock topics using production split limits:

```bash
ros2 launch inspection_bringup record_only.launch.py \
  debug_mode:=false output_directory:="$PWD/bags/hardware_run"
```

Inspect and validate a closed MCAP bag:

```bash
ros2 bag info -s mcap bags/mock_run
ros2 run inspection_tools validate_bag bags/mock_run \
  --output reports/mock_run_validation.json
```

Hash, manifest and copy a validation-passed bag without deleting the local original:

```bash
ros2 run inspection_tools finalize_bag bags/mock_run \
  --task-id task-001 --asset-id tank-001 --course-id course-00 --plate-id plate-00 \
  --validation-report reports/mock_run_validation.json --remote-store remote_store
```

Replay raw and recorded derived topics (storage is read from rosbag2 metadata):

```bash
ros2 bag play bags/mock_run
```

For the current USB2.1 MID-360 + D405 baseline loop, capture a clean 60-second raw bag with
one command (it stops and flushes automatically):

```bash
ros2 launch inspection_bringup baseline_capture.launch.py
```

Then replay the newest baseline bag, reconstruct a downsampled D405 XYZ cloud from the recorded
depth image and CameraInfo, and inspect it together with MID-360, the recorded color image and TF:

```bash
ros2 launch inspection_bringup baseline_replay.launch.py
```

The replay cloud is derived and is not written into the raw MCAP. To select an older closed bag,
pass `bag_directory:=/absolute/path/to/baseline_loop_*`; `point_stride:=1` restores every valid
depth pixel when the host has sufficient rendering capacity. The transform used at this stage is
the explicitly non-production rough transform in `rough_extrinsics.yaml`.

With the provisional Eddy device connected at `/dev/ttyACM0`, capture a 60-second
Eddy-anchored time-matching bag with:

```bash
ros2 launch inspection_bringup eddy_sync_capture.launch.py
```

This mode records `EddyCurrentFrame` and derived `FusionIndex` together with D405, MID-360 and
IMU. It does not invent unavailable Eddy Q/sampling/calibration metadata, and pose remains invalid
until `/robot/odometry` is connected. Validate this engineering-stage bag using the Eddy demo
profile (`validate_bag --eddy-demo-profile --allow-provisional-eddy`), or simply run
`ros2 run inspection_tools inspect_latest`; omit the provisional flag for strict production
acceptance. All
approximately 500 Hz Eddy frames remain raw data, while every tenth frame anchors a roughly 50 Hz
`FusionIndex`, preventing derived-index backlog without discarding source measurements. The
initial software-time loop uses bounded caches and selects the nearest D405/MID-360 header stamp
for each Eddy anchor. It leaves every raw header stamp unchanged and reports the selected stamps,
time errors and validity flags in `FusionIndex`.

The default camera and Livox clock offsets are zero and online offset adaptation is disabled.
Tests on the accepted bag showed that fitting a constant offset to periodic 15 Hz/10 Hz streams
can merely switch which frame is selected without reducing absolute timing error. Fixed offsets
remain optional engineering inputs (`camera_clock_offset_ms` and `livox_clock_offset_ms`) for a
future shared-event calibration; they should not be tuned from nearest-frame error alone.

To prove regeneration, run `inspection_sync_node` separately and exclude the recorded index:

```bash
ros2 run inspection_sync inspection_sync_node \
  --ros-args --params-file src/inspection_bringup/config/cache.yaml
ros2 bag play bags/mock_run --rate 2.0 \
  --exclude-topics /derived/inspection/fusion_index /diagnostics
```

The mock pipeline uses `asset_coordinate_mode: tank_xyz`: odometry is a Cartesian pose in
`tank_course`, and the synchronizer converts it to cylindrical `(s_m, v_m, n_m)` using
`asset_radius_m`, `asset_start_angle_rad`, and `asset_pose_standoff_m`. The mock sensor
`tank_radius_m`, `tank_start_angle_rad`, and `robot_standoff_m` values must match those sync
parameters and the tank schematic. Production configurations may retain
`asset_coordinate_mode: unwrapped_pose` when their localization source already publishes
`(s, v, n)` in the odometry position fields.

The mock robot can execute a portable mission trajectory exported by `tank_vision_inspection`
instead of the fallback helix. Start the bridge first, then the mock pipeline in another shell:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv_rerun/bin/activate

ros2 launch inspection_rerun rerun_live.launch.py \
  output_mode:=save \
  save_path:="$PWD/output/rerun/mock_mission_fused.rrd" \
  log_tank_scene:=true \
  tank_asset_path:=/absolute/path/to/tank_schematic.yaml \
  mission_path:=/absolute/path/to/mission_schematic.json
```

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch inspection_bringup mock_pipeline.launch.py \
  record:=false \
  mission_trajectory_path:=/absolute/path/to/mission_mock_trajectory.json \
  mission_speed_m_s:=2.0 mission_dwell_s:=0.5 mission_loop:=true
```

`mission_speed_m_s`, `mission_dwell_s`, and `mission_loop` are launch arguments whose defaults
come from `src/inspection_bringup/config/mock_sensors.yaml`. Mission reposition points are
instantaneous discontinuities; the Rerun actual-trajectory logger intentionally does not connect
those jumps.

Run tests and estimate uncompressed storage:

```bash
colcon test
colcon test-result --verbose
ros2 run inspection_tools estimate_storage --json --task-duration-minutes 60
```

## Hardware boundary

`hardware_pipeline.launch.py` now starts the MID-360, connected D405 depth/color streams,
SI-unit IMU adapter and production MCAP recorder by default. The output is automatically assigned
a new workspace directory such as
`bags/mid360_d405_run_YYYYMMDD_HHMMSS_microseconds`, so the normal command is simply:

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py
```

The connected D405 is selected automatically, so no serial argument is required. A serial selector
remains available only when multiple RealSense devices are attached and is never stored as a
default. Both `start_mid360` and `start_d405` default to true; eddy-current and synchronization
remain disabled until their hardware/time contracts are accepted. MID-360 uses the local
`livox_ros_driver2` package in `xfer_format=0` PointXYZRTLT mode and requires an explicit vendor
JSON path. The currently verified site JSON is under the vendor package and must be reviewed for
each host/network deployment. The PointCloud2 contract is
`x/y/z/intensity/tag/line/timestamp`; an adapter may instead provide a documented `time_offset`
field. The vendor IMU Topic is retained as `/livox/imu_raw` (acceleration in g), while
`inspection_livox_adapter` publishes `/livox/imu` with acceleration in m/s^2 and unchanged
rad/s angular velocity. The eddy driver publishes every parsed 8x20 ADC frame on
`/inspection/eddy_current/raw` as well as its legacy status/debug topics.

To inspect both devices without recording:

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  record:=false
```

The current STM32 protocol has no device timestamp or Q component. The driver therefore marks
the standard output conservatively: `header.stamp` is host frame-receive completion time,
`signal_q` is NaN, and `quality_score` is zero. These facts are also published on `/diagnostics`;
they must be resolved before calibrated multimodal fusion is accepted. The synchronizer defaults
to the eddy-current topic as its anchor, but does not start unless `start_sync:=true`. Hardware
asset fields default to `UNASSIGNED`, and `asset_coordinate_valid` remains false until calibration
and survey mapping.

MCAP files are treated as immutable raw sources. `FusionIndex` contains only timestamps,
validity, temporal errors, interpolated pose and asset coordinates—never raw image, cloud or
waveform payloads.
