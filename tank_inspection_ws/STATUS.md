# Tank inspection acquisition status

Updated: 2026-07-21 (Asia/Shanghai)

## Environment

- Host: Ubuntu 24.04.3 LTS, x86_64, Linux 6.14.0-37-generic.
- ROS: Jazzy; `/opt/ros/jazzy/bin/ros2`; `rosbag2_storage_mcap` 0.26.9.
- Python: system `/usr/bin/python3` 3.12.3 for all ROS work. Conda `base` Python 3.13.9 was
  deliberately excluded.
- Disk before implementation: about 140 GiB free.
- Source root: `/home/dts/physic_bev/tank_inspection_ws` (the session's writable workspace).
- The user installed `ros-jazzy-realsense2-camera` and upgraded
  `ros-jazzy-diagnostic-updater` with sudo. The acquisition work made no system-network changes,
  Git initialization, public-cloud upload, credential storage, user-file deletion, or local
  raw-data deletion.
- Livox ROS Driver 2 source version 1.2.6 is now part of this workspace. The host has Livox SDK2
  1.2.5 in `/usr/local`; a compile-time guard keeps ordinary MID-360 support compatible while
  excluding the SDK-1.2.6-only MID-360S enum.

## Build and tests

- `colcon build --symlink-install`: PASS, 8 packages.
- `colcon test`: PASS.
- `colcon test-result --verbose`: **26 reported tests, 0 errors, 0 failures, 0 skipped**.
- Coverage exercised: count bound, age eviction, out-of-order insertion, nearest camera frame,
  bounded software clock-offset math, translation interpolation, quaternion SLERP,
  missing/timeout flags, ultrasound array shape,
  PointCloud2 required fields, D405 depth-image projection/stride/padding, manifest generation and
  SHA256 stability, and Livox IMU SI-unit conversion/rejection behavior.
- The vendor package's upstream style-only lint suite is opt-in through
  `LIVOX_ROS_DRIVER2_ENABLE_LINT_TESTS=ON`; it is disabled by default because the unmodified vendor
  sources and bundled RapidJSON currently fail those linters. No vendor runtime tests were hidden.

## Final end-to-end acquisition

- Bag: `bags/mock_e2e_final_20260717_1507`.
- Clean SIGINT closure: PASS; recorder flushed its cache and all three processes exited 0.
- Duration: 104.809748492 s.
- Storage: MCAP/Zstd with chunking, CRC, message index and summary enabled.
- Automatic debug splitting: PASS; 4 MCAP files (30 s limit), total 514,184,049 bytes
  (rosbag2 reports 490.4 MiB).
- Topics: 13; messages: 56,169.
- Configured rates measured from header timestamps:
  - depth Image + CameraInfo: 15.000 Hz, 1,572 each;
  - lidar: 10.000 Hz, 1,048;
  - IMU: 199.990 Hz, 20,959;
  - odometry: 49.990 Hz, 5,239;
  - ultrasound anchor: 50.000 Hz, 5,240;
  - eddy current: 24.990 Hz, 2,620;
  - probe state: 49.981 Hz, 5,238;
  - robot state: 20.000 Hz, 2,096.
- Validation: PASS, 0 structural or timestamp errors.
- Fusion: 5,240 indices; 5,239 fully valid (99.9809%). Camera, lidar and IMU were 100%; the
  single invalid pose is the expected startup boundary before a two-sided odometry bracket exists.
- Maximum absolute temporal errors: camera 33.569 ms; lidar 42.512 ms, both within tolerance.
- Diagnostics: late=0, out-of-order=0, rejected=0, pending_dropped=0. Cache length peaks were
  camera 31/100, lidar 21/40, IMU 1001/1500, odometry 251/500, ultrasound 101/200,
  eddy 51/200, robot 101/500, probe 251/500. No unbounded-growth symptom was observed.
- No observable acquisition-rate loss in the final bag: measured rates match configured rates;
  boundary counts differ only with Topic phase and recorder start/stop time.

### Resource samples

Five-second `pidstat` samples on the same improved configuration showed:

- mock publishers: about 70–85% of one CPU core, RSS ~41.6 MiB;
- synchronization: about 11–12% of one CPU core, RSS ~61–64 MiB;
- MCAP recorder: about 20–23% of one CPU core, RSS ~84–93 MiB;
- recorder disk writes: about 4.77–4.81 MiB/s;
- no major faults, swap use, rising cache RSS, or I/O delay.

GNU `time` for the final run reported 114% aggregate CPU, 92,544 KiB maximum resident set for
the measured launch command, 1:45.95 wall time and exit status 0.

## Validation, manifest and copy

- Validation report: `reports/mock_e2e_final_20260717_1507_validation.json`.
- Manifest: `bags/mock_e2e_final_20260717_1507/manifest.yaml`.
- Final lifecycle: `CLOSED -> HASHED -> COPIED -> VERIFIED`.
- Verified copy:
  `remote_store/mock-task-20260717/mock_e2e_final_20260717_1507`.
- Local and copied SHA256 values match for all four MCAP files:
  - `_0.mcap`: `2d135d917eb13c14436dc0834ec031f1f53a6f677e0f3d5dfc73c1eb3179fe67`
  - `_1.mcap`: `6e082e9b253cf9fe156c381a9231754ed2c891f10bf1a3f3e6d16d4354969b09`
  - `_2.mcap`: `b552760d4beb746c23950545452c9716f125444ff33ecbd6a61619138158ca2e`
  - `_3.mcap`: `fec451522f0b02108a319a75b0388c9030fe3e104fb0e70eb46f9e16a345ea4d`

## Playback verification

- Full 104.81 s bag replayed at 2x with exit code 0.
- Recorded `/derived/inspection/fusion_index` and `/diagnostics` were excluded.
- A fresh `inspection_sync` produced exactly 5,240 new FusionIndex messages from 5,240 anchors.
- Derived verification bag: `bags/replay_regenerated_20260717` (578.8 KiB).
- Derived validation report: `reports/replay_regenerated_20260717_validation.json`, PASS/0 errors.
- Replay fusion full-valid rate: 99.9046%; IMU/lidar 100%, camera/pose 99.9427%. Five invalid
  boundary/BestEffort cases correctly carry invalid flags and reasons; no zero data is marked valid.
- During playback, diagnostic `oldest_age_s` compares original recorded stamps with current wall
  time, so its large absolute value is expected; cache lengths remain bounded by stamp span/count.

## Storage estimate

Default uncompressed payload estimate: 13.1008 MB/s, 47.16288 GB/h, and 47.16288 GB for a
60-minute task. It intentionally excludes protocol overhead and does not assume an MCAP
compression ratio.

## Real D405 hardware smoke test

- Device: Intel RealSense D405; firmware 5.15.1.55; RealSense ROS 4.58.1 and librealsense 2.58.1.
- The initial driver load exposed a package ABI mismatch; upgrading
  `ros-jazzy-diagnostic-updater` from 4.2.6 to 4.2.7 resolved it. No driver source patch was used.
- Connection: detected as USB 2.1 through the current USB path; the driver warns that reduced
  performance is possible. A direct USB 3.x connection is still recommended for sustained work.
- Tested stream: depth `640x480x30`, `16UC1`, `camera_depth_optical_frame`; `step=1280` and
  `data` length 614,400 bytes are structurally correct.
- Live rate: approximately 29.59--30.03 Hz; measured subscriber bandwidth approximately
  18.38 MB/s. One observed scheduling interval reached 68 ms, without timestamp regression.
- Closed bag: `bags/d405_smoke_20260717_1558`, duration 54.3128584 s, 3,318 messages, 4 Topics,
  two automatic debug splits, 320,097,805 MCAP bytes (rosbag2: 305.3 MiB).
- Image count/rate: 1,631 / 30.0302 Hz. CameraInfo count/rate: 1,632 / 30.0302 Hz.
  Diagnostics: 54 / 1.0000 Hz. All recorded Topic timestamp regression counts are zero.
- Partial hardware validation: PASS with 0 errors. This bag intentionally contains only D405,
  diagnostics and static TF; it is not presented as a complete multimodal hardware test.
- Validation report: `reports/d405_smoke_20260717_1558_validation.json`.
- Manifest: `bags/d405_smoke_20260717_1558/manifest.yaml`; lifecycle
  `CLOSED -> HASHED -> COPIED -> VERIFIED`.
- Verified copy: `remote_store/d405-smoke-20260717/d405_smoke_20260717_1558`; both MCAP SHA256
  values match their local originals. Asset/course/plate remain explicitly `UNASSIGNED`.
- Playback: PASS. A reliable subscriber established before 1x playback received a structurally
  correct depth frame with no loss warning; playback and subscriber exited normally.
- `hardware_pipeline.launch.py`: real-device PASS. `start_d405:=true` starts only the official
  D405 driver node when `start_mid360:=false`; `start_sync` remains opt-in while recording now
  defaults on. The serial number is a launch argument, not a stored default.
  Unsupported-parent-parameter warnings were removed by passing only documented RealSense node
  parameters.
- Hardware synchronizer parameter readback: anchor `/inspection/eddy_current/raw`, asset ID
  `UNASSIGNED`, `asset_coordinate_valid=false`, and reliable camera QoS. Clean SIGINT exit: PASS.

## Real MID-360 hardware smoke test

- Network/device: configured MID-360 at `192.168.2.3`, host data interface
  `192.168.2.100`; ping had 0% packet loss and the driver received real UDP point/IMU streams.
- Driver: local `livox_ros_driver2` 1.2.6, `xfer_format=0`, `publish_freq=10.0`, frame
  `livox_frame`; startup and SDK deinitialization both completed cleanly.
- Live PointCloud2: 10.0001 Hz; observed frame width 19,968, `height=1`, `point_step=26`,
  `row_step=data_length=519,168`. Fields and offsets were `x:0`, `y:4`, `z:8`,
  `intensity:12`, `tag:16`, `line:17`, and float64 `timestamp:18`.
- One sampled cloud covered 100.25344 ms of per-point timestamps. Serialized point order had 137
  local timestamp regressions because scan lines are interleaved; consumers must use each point's
  timestamp rather than assume the byte array is time-sorted.
- The official MID-360 protocol defines gyro in rad/s and acceleration in g. The vendor driver
  copies both values unchanged into `sensor_msgs/Imu`, so the project now preserves that output as
  `/livox/imu_raw` and publishes ROS-SI `/livox/imu` through `inspection_livox_adapter`.
- Live IMU: raw and SI Topics both 199.9937 Hz with identical timestamp sets. Stationary
  acceleration norm averaged 0.99334 g raw and 9.74130 m/s^2 after the 9.80665 conversion.
  Canonical orientation is explicitly marked unavailable with `orientation_covariance[0]=-1`.
- Adapter diagnostics after the live run: rejected=0 and non-monotonic=0. The recorded diagnostic
  ended at received=published=18,555 with the same zero error counts.
- Closed bag: `bags/mid360_smoke_20260721_1525`, duration 92.55454343 s, 37,765 messages,
  four automatic debug splits, 217.5 MiB. Counts/rates were lidar 905 / 9.99994 Hz,
  raw IMU 18,100 / 200.00267 Hz, canonical IMU 18,667 / 199.50028 Hz, and diagnostics 93 /
  1.00000 Hz. The initial IMU count difference is recorder subscription startup order, not adapter
  loss; the live simultaneous check had equal timestamps.
- Partial hardware validation: PASS with 0 errors and all Topic header regression counts zero.
  Report: `reports/mid360_smoke_20260721_1525_validation.json`.
- Manifest: `bags/mid360_smoke_20260721_1525/manifest.yaml`; lifecycle
  `CLOSED -> HASHED -> COPIED -> VERIFIED`. The verified copy is
  `remote_store/mid360-smoke-20260721/mid360_smoke_20260721_1525`; SHA256 matches for all four
  MCAP files, and the local original remains present.
- Bare-command default-profile check: `ros2 launch inspection_bringup hardware_pipeline.launch.py`
  automatically selected the installed JSON and a unique output directory. The resulting
  `bags/mid360_run_20260721_154527_547554` is 29.364 s / 11,603 messages / 67.4 MiB and passed
  partial validation with 0 errors.
- Updated bare-command joint profile check: the same command automatically selected the connected
  D405 and MID-360 and recorded `bags/mid360_d405_run_20260721_161500_970746`. The bag is
  61.987 s / 28,792 messages / 314.4 MiB and passed partial validation with 0 errors. Measured
  rates were depth Image and CameraInfo 30.0269 Hz, lidar 10.0002 Hz and raw IMU 199.9963 Hz;
  every recorded Topic had zero header timestamp regressions. Report:
  `reports/mid360_d405_run_20260721_161500_970746_validation.json`.
- This is a MID-only acquisition check, not a calibrated fusion result. The current JSON
  extrinsics are all zero; base-to-lidar calibration, cross-device clock validation, disconnect/
  reconnect behavior and sustained thermal/network/disk testing remain open.

## Real MID-360 + D405 baseline loop

- Closed bag: `bags/baseline_loop_20260721_202315_810997`; the time-bounded launch stopped after
  60 seconds and all sensor, TF and recorder processes exited cleanly.
- Bag result: 59.427841417 s, 27,577 messages, one MCAP file, 999.5 MiB.
- Recorded rates: D405 color 15.01883 Hz, D405 depth 15.01565 Hz, MID-360 cloud 9.98237 Hz,
  Livox raw IMU 200.00271 Hz and SI IMU 199.06975 Hz. These Topics had zero header timestamp
  regressions.
- Partial hardware validation: PASS with 0 errors. Report:
  `reports/baseline_loop_20260721_202315_810997_validation.json`.
- The closed bag contains three transient-local `/tf_static` messages. Headless replay recovered
  `livox_frame -> camera_link` as translation `(0.055, -0.008, -0.068) m`, quaternion
  `(1, 0, 0, 0)` / roll 180 degrees, matching rough calibration revision 02.
- Replay reconstruction from the recorded depth image plus CameraInfo produced a valid 56,089-point
  XYZ cloud at the default two-pixel stride. The newest-baseline auto-selection path and clean
  replay shutdown both passed. Visual overlap remains a human acceptance check because the
  transform is deliberately rough and `production_valid=false`.

## Real provisional Eddy-anchored time loop

- Device: `/dev/ttyACM0`, 4 Mbaud, STM32 8-channel protocol. Hardware queries returned amplitude
  450 and excitation frequency 450,000 Hz. Each canonical frame contains 8 channels x 20 samples.
- Standalone diagnostics observed about 500 parsed frames/s and zero serial queue drops. The
  protocol still has no device timestamp, Q component, confirmed per-channel sampling rate, gain,
  lift-off or calibration identity; these remain explicit provisional-validation warnings.
- The initial 60-second engineering run preserved all raw Eddy frames but revealed that a DDS
  subscription depth of five could not reliably consume the driver's 50-frame bursts. The sync
  subscriber depth is now 200, and every tenth Eddy frame is used as a derived synchronization
  anchor while every raw frame remains recorded.
- Accepted verification bag: `bags/eddy_sync_loop_20260721_211217_658866`; 29.358692831 s,
  29,645 messages, one 489.9 MiB MCAP file. Provisional structural validation passed with zero
  errors and no Topic timestamp regressions. Report:
  `reports/eddy_sync_loop_20260721_211217_658866_validation.json`.
- Exact raw/index ratio: 14,700 Eddy frames at 499.9760 Hz and 1,470 FusionIndex messages at
  49.9971 Hz. Eddy driver dropped frames=0; synchronizer pending_dropped=0; anchor decimation=10.
- Ignoring the unavailable robot pose, camera+lidar+IMU temporal validity was 1,345/1,470 overall
  because MID-360 needed about 2.5 seconds to begin publishing. After the first complete temporal
  match, steady-state validity was 1,345/1,345 (100%).
- Relative to Eddy host-receive completion stamps, D405 nearest-frame error had signed mean
  -0.039 ms, mean absolute 16.668 ms, p95 absolute 31.693 ms and maximum absolute 36.139 ms.
  MID-360 nearest-cloud error had signed mean -1.397 ms, mean absolute 24.281 ms, p95 absolute
  41.623 ms and maximum absolute 42.048 ms. These are software timestamp-matching results, not a
  hardware-trigger or device-clock calibration claim.
- Software clock matching is therefore implemented as phase-safe bounded nearest-stamp matching,
  with raw sensor stamps immutable, fixed correction defaulting to zero and continuous offset
  adaptation disabled. A deterministic replay control produced 99.926% post-start temporal
  validity. A +1.397 ms Livox fixed-offset trial centered signed error (+8.242 ms to -0.146 ms in
  that replay) but worsened mean absolute error (25.948 ms to 26.170 ms) and post-start validity
  (99.926% to 99.778%); it was rejected as periodic-frame phase switching rather than a real clock
  improvement. Control report: `reports/eddy_clock_control_replay_20260721_2134_validation.json`;
  rejected-offset evidence: `reports/eddy_clock_static_replay_20260721_2132_validation.json`.
- `/robot/odometry` was absent, so `pose_valid=0` and fully-valid fusion rate remains zero by
  design. Two transient D405 ASIC-temperature query errors appeared during startup, but both image
  streams maintained about 15 Hz and passed structural/timestamp validation.

## Completed

- Chinese operator runbook `DEMO_WORKFLOW_ZH.md`, latest-bag `inspect_latest` validation summary,
  Eddy-demo-specific required Topic profile, and one-command latest Eddy MCAP + RViz replay.
- Independent raw Topics and custom interface package.
- Deterministic configurable mock depth geometry, tank-wall cloud, IMU/trajectory, robot/probe
  state, dual-channel A-scan and eddy-current signals, plus `/tf` and `/tf_static`.
- C++ timestamp-sorted bounded caches with both age and count limits; asynchronous fusion worker.
- Camera/lidar matching, IMU windowing, odometry translation interpolation and quaternion SLERP.
- FusionIndex-only derived output and detailed diagnostics.
- MCAP debug/production profiles, duration/size splitting, QoS overrides and launch modes.
- Closed-bag validation, SHA256 manifest lifecycle, verified local NAS-style copy, storage estimate,
  and playback regeneration.
- Integrated the newly added `eddy_driver` into the canonical acquisition path. It preserves the
  legacy state/debug topics, drains parsed ADC frames in order to `/inspection/eddy_current/raw`,
  bounds and reports queue drops, and publishes explicit diagnostics for provisional timestamp and
  I-only semantics.
- Set the MID-360 + connected D405 depth/color + IMU adapter + production MCAP recorder as the default
  `hardware_pipeline.launch.py` profile. A bare launch uses the installed vendor JSON and creates
  a new timestamped bag directory automatically; eddy-current and synchronization remain explicit
  opt-ins.
- Added `inspection_livox_adapter`, preserving raw-g IMU data while providing the ROS SI-unit IMU
  contract and conversion diagnostics; both Topics are now included in the MCAP profile.
- Extended validation for eddy array/metadata shape and both official Livox per-point `timestamp`
  and adapter-provided `time_offset` PointCloud2 contracts.

## Not yet hardware-validated / next steps

- D405 depth/color and MID-360 acquisition have separate and joint real-device smoke tests. Ultrasound,
  eddy-current and the robot controller have not completed real-device validation; no complete
  multimodal hardware fusion claim is made.
- The current site D405 link is constrained to USB 2.1. The baseline loop therefore uses explicit
  640x480x15 depth/color profiles, matching the measured closed-bag rates; 30 Hz remains a later
  USB-link upgrade and re-validation target.
- Added a traceable rough `livox_frame -> camera_link` transform for baseline-loop visualization:
  translation `(0.055, -0.008, -0.068) m`, reported forward alignment `+X`, reported camera top
  `-Z` (`roll=pi`). The Y value includes an 18 mm D405 imager-baseline correction after the
  front-view right lens was identified as the initial measurement reference instead of the left
  imager used by `camera_link`. It is explicitly marked `production_valid=false` and must be
  replaced by a target-based calibration before metrology or defect localization.
- Added a bounded baseline-loop workflow. `baseline_capture.launch.py` records 60 seconds by
  default and cleanly shuts down; `baseline_replay.launch.py` automatically selects the newest
  closed baseline bag, loops it, regenerates a lightweight D405 XYZ cloud from raw depth plus
  CameraInfo, and displays it with MID-360, recorded color and TF. The derived cloud is not added
  to the raw MCAP, and the default two-pixel stride limits replay/rendering load.
- The installed Livox SDK2 is 1.2.5 while the driver is 1.2.6. Ordinary MID-360 operation is
  verified with the compatibility guard, but production deployment should align both to one
  pinned version and rebuild/retest.
- The current STM32 eddy protocol exposes neither a device timestamp nor confirmed I/Q semantics.
  The integrated driver uses host frame-receive completion time, publishes I-only ADC samples with
  NaN Q and quality zero, and must not be treated as calibrated fusion data yet.
- No Physics-BEV model, training, inference or performance claim exists in this phase.
- Before hardware acquisition:
  1. calibrate D405 intrinsics/depth scale and all base-to-camera/lidar/probe extrinsics;
  2. calibrate MID-360 extrinsics and document the interleaved per-point timestamp ordering;
  3. choose and verify PTP/NTP/hardware-trigger time synchronization and clock-drift monitoring;
  4. calibrate ultrasound velocity/wedge delay/gain and eddy excitation/gain/lift-off;
  5. calibrate contact-force sensing and define coupling/contact acceptance thresholds;
  6. validate tank course/plate/weld asset coordinates and pose confidence against survey data;
  7. run sustained hardware throughput, thermal, network-loss and disk-capacity tests before use.
- The local vendor package generates Livox CustomMsg types, but the core acquisition path does not
  depend on them. MID-360 PointCloud2 replaces the mock Topic without changing the synchronizer.
