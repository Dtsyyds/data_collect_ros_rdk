# Rerun portability: x86 Jazzy to S100P Humble

## Code that should transfer unchanged

The `inspection_rerun` Python package, ROS parameters, entity paths, integer-nanosecond
timeline, image/point-cloud adapters, deterministic sampling, message-level error
handling, and launch argument model are platform-neutral. They use Python 3.10 syntax
and ROS 2 APIs shared by Humble and Jazzy. `inspection_interfaces` definitions must
remain identical on both systems; do not translate or extend fields only for Rerun.

## Environment-specific work on S100P

1. Source the vendor ROS 2/TROS Humble setup instead of `/opt/ros/jazzy/setup.bash`.
2. Select the interpreter to which Humble `rclpy` is actually bound (normally Python
   3.10 on Ubuntu 22.04) and prove it with `import rclpy` before creating a venv.
3. Create the venv with that interpreter and `--system-site-packages`.
4. Confirm `rerun-sdk==0.36.2` provides a compatible Linux ARM64 wheel and that the
   S100P OS/glibc meets its wheel requirements. Do not substitute an x86 wheel.
5. Replace `scripts/setup_rerun_local.sh`; it intentionally rejects non-x86_64 and
   checks the local Jazzy path. Platform checks belong in setup scripts, not node code.
6. Build and source `inspection_interfaces` before `inspection_rerun`.
7. Adjust YAML topic names only if S100P publishes different names.
8. Prefer `output_mode=save` for initial validation. Native Viewer availability and
   GPU support on S100P are separate from ROS bridging and RRD generation.

## Compatibility verification

Compare every `.msg` file with the capture-side source before replay. Run
`ros2 bag info`, then replay a short time range. If Jazzy/Humble or TROS cannot load a
custom type, report the exact missing type support rather than reindexing or altering
the MCAP. Validate standard topics first, then custom messages, then GUI rendering.

No migration step may put Rerun into the acquisition path. MCAP remains the raw-data
source and the bridge may be stopped without affecting recording or playback.
