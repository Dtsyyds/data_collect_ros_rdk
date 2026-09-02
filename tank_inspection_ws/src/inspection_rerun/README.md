# inspection_rerun

`inspection_rerun` is an optional ROS 2 visualization bridge for local development,
MCAP replay, and regression inspection. It mirrors selected ROS messages into Rerun
and can spawn a Viewer, connect to an existing Rerun gRPC endpoint, or write an
`.rrd` without a GUI.

MCAP remains the immutable source of raw measurement data. This package is not in
the acquisition path, does not record bags, and is not a dependency of any camera,
LiDAR, IMU, eddy-current, synchronization, or vision algorithm.

## Local Jazzy installation

The local machine uses ROS 2 Jazzy's system Python 3.12. The currently installed
Conda Python 3.13 cannot load Jazzy's Python 3.12 `rclpy` extension. From the
workspace root:

```bash
./scripts/setup_rerun_local.sh
source /opt/ros/jazzy/setup.bash
source .venv_rerun/bin/activate
```

The script uses `/usr/bin/python3 -m venv --system-site-packages` and fixes one
coherent Python 3.12 set: `rerun-sdk==0.36.2`, `numpy==2.2.6`, and the
NumPy-2-compatible `opencv-python-headless==4.12.0.88`. The bridge itself does
not import OpenCV or the separate `tank_vision` project. The wheel-provided
`cv2` therefore takes precedence over Ubuntu's NumPy-1.x-built system extension
without changing system packages. The script uses no `sudo` and does not edit
shell startup files.
The virtual environment must remain active when launching the node; the package's
wrapper deliberately checks that both `rclpy` and `rerun` import successfully.

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
source .venv_rerun/bin/activate
cd /path/to/tank_inspection_ws
python -m compileall -q src/inspection_rerun scripts
python -m pytest -q src/inspection_rerun/test
colcon build --symlink-install --packages-select inspection_interfaces inspection_rerun \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`colcon test` itself starts the system pytest executable. To let that executable
see the venv-only Rerun package, expose the two pinned SDK directories explicitly:

```bash
rerun_site="$VIRTUAL_ENV/lib/python3.12/site-packages"
PYTHONPATH="$rerun_site:$rerun_site/rerun_sdk${PYTHONPATH:+:$PYTHONPATH}" \
  colcon test --packages-select inspection_rerun
```

## Smoke tests

Headless save mode is the automated default:

```bash
python scripts/rerun_viewer_smoke_test.py --output-mode save
rerun output/rerun/smoke_test.rrd
```

To test the native Viewer:

```bash
python scripts/rerun_viewer_smoke_test.py --output-mode spawn
```

A graphics failure does not affect `save` mode. Do not change GPU drivers or
inject Mesa/Vulkan libraries through `LD_LIBRARY_PATH` to work around Viewer issues.

## Live topics and MCAP replay

Live visualization starts only the bridge:

```bash
ros2 launch inspection_rerun rerun_live.launch.py output_mode:=spawn
```

Bounded replay starts the bridge and `ros2 bag play`, never `ros2 bag record`:

```bash
ros2 launch inspection_rerun rerun_replay.launch.py \
  bag_path:=/absolute/path/to/bag.mcap output_mode:=spawn \
  start_offset:=0.0 play_duration:=10.0 playback_rate:=1.0
```

Headless RRD output:

```bash
ros2 launch inspection_rerun rerun_replay.launch.py \
  bag_path:=/absolute/path/to/bag.mcap output_mode:=save \
  save_path:=output/rerun/replay.rrd play_duration:=10.0
```

Tank + planned mission + actual FusionIndex trajectory in one recording:

```bash
ros2 launch inspection_rerun rerun_replay.launch.py \
  bag_path:="$PWD/bags/mock_demo_virtual_20260901" \
  output_mode:=save save_path:="$PWD/output/rerun/mock_tank_fused.rrd" \
  log_tank_scene:=true \
  tank_asset_path:=/absolute/path/to/tank_schematic.yaml \
  mission_path:=/absolute/path/to/mission_schematic.json
rerun "$PWD/output/rerun/mock_tank_fused.rrd"
```

Replay arguments are `bag_path`, `config_file`, `output_mode`, `save_path`,
`playback_rate`, `playback_delay`, `loop`, `start_offset`, `play_duration`, and `use_clock`.
Scene arguments are `log_tank_scene`, `tank_asset_path`, and `mission_path`.
`bag_path` must be absolute and existing. Looping and `/clock` are off by default.
The default two-second playback delay lets the bridge establish subscriptions before
the first bag message.

## Topics, entities, and performance controls

All topic names are parameters in `config/rerun.yaml`. Standard support includes
`Image`, `CameraInfo`, `PointCloud2`, `Imu`, `TFMessage`, `DiagnosticArray`,
`Odometry`, `Path`, and `PoseStamped`. Workspace-defined support includes
`EddyCurrentFrame`, `UltrasoundFrame`, `ProbeState`, `RobotState`, and
`FusionIndex`. Missing topics are harmless: the bridge simply records no entities
for a stream that is not present in a live system or MCAP.

The historical topic name
`/camera/camera/depth/image_rect_raw` contains MindVision `mono8` intensity images,
not depth measurements in older hardware bags. Supported image encodings are
`mono8`, `mono16`, `16UC1`, `rgb8`,
`bgr8`, `rgba8`, and `bgra8`, including padded rows. Image limiting occurs before
large-buffer copying or color conversion. Default limits are 2 Hz for images,
1 Hz and 100,000 displayed points for clouds, and 20 Hz per IMU topic. Point stride
and optional voxel size are configurable. Cloud maximum-count sampling is
deterministic.
`16UC1` is logged as `DepthImage`; `depth_units_per_meter` defaults to 1000 for
the mock/D405 millimetre contract.

Valid `CameraInfo` is logged as a pinhole model. All-zero intrinsics produce one
warning and no pinhole; values are never fabricated. TF handles dynamic and static
topics separately, normalizes valid quaternions, and allows disconnected frame trees.
When `/tf` logging is enabled it is authoritative for odometry transforms, preventing
`base_link` from being logged by two temporal transform entities. Point clouds are assigned
their ROS `header.frame_id`, and the pinhole uses the optical frame as its 3D parent.

The `FusionIndex` synchronization entity contains associations, validity, time errors,
quality, and identifiers. When `log_tank_scene` is enabled, each valid index also
maps `(s_m, v_m, n_m)` to the same tank model: `s_m` is circumferential distance,
`v_m` is height, and `n_m` is outward wall-normal offset. Both the current point and
actual path are logged as paired `Points3D`/`Points2D` and
`LineStrips3D`/`LineStrips2D` entities. Invalid coordinates and heights outside the
tank are reported and omitted from the spatial path.

All actual messages use the `ros_time` timestamp plus `frame_sequence`. Static tank
and planned mission entities have no execution timestamp. This bridge never writes
`inspection_step`; that sequence remains exclusive to the standalone deterministic
planning-preview animation. Images, clouds, eddy current, ultrasound, probe state,
robot state, synchronization status, and the tank scene all share one recording.
Images and clouds remain on their source entities. In the current
interface, eddy frames have only the ROS header timestamp; they do not define raw
device time, corrected time, clock offset, or drift. `FusionIndex` defines camera and
LiDAR errors but no eddy or IMU delta. The bridge does not invent absent fields.

## Humble MCAP on Jazzy

Run `ros2 bag info` first. A Humble MCAP containing standard CDR messages can often
be replayed by Jazzy, but message type support must exist locally. The Jazzy
`inspection_interfaces/*.msg` definitions must be byte-for-byte interface-compatible
with the S100P capture definitions. A missing type support library is a dependency
problem, not proof of MCAP corruption. Do not reindex unless the bag is known to be
incomplete and the operator explicitly chooses to do so.

## S100P Humble migration

The Python modules use Python 3.10-compatible syntax and avoid Jazzy-only core APIs.
Copy the package, YAML, requirements, and smoke script. Replace the local x86/Jazzy
environment setup with an ARM64 ROS 2/TROS Humble setup, confirm that the pinned
Rerun wheel is available for that platform, and build the identical interface
definitions. See `docs/RERUN_PORTABILITY.md`.

## Troubleshooting and known limits

- Activate the Rerun virtual environment before invoking a launch file.
- If `inspection_interfaces` cannot import, build it and source the workspace.
- Zero camera intrinsics prevent accurate projection but do not block image display.
- Missing TF links remain separate trees instead of being guessed.
- One unsupported or malformed message is counted, reported, and skipped; the node
  does not modify the bag or stop `ros2 bag play`.
- Viewer graphics problems are independent of headless `.rrd` generation.
- Detection boxes, segmentation masks, and physical BEV entities are reserved paths;
  adapters for those algorithm outputs are not implemented yet.
