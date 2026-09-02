#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${workspace_dir}/.venv_rerun"
ros_setup="/opt/ros/jazzy/setup.bash"
system_python="/usr/bin/python3"

if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "error: this local setup script supports x86_64 only" >&2
  exit 1
fi

if [[ ! -r "${ros_setup}" ]]; then
  echo "error: ROS 2 Jazzy setup not found: ${ros_setup}" >&2
  exit 1
fi

# ROS setup references variables that may be unset in a clean shell.
set +u
source "${ros_setup}"
set -u

if ! "${system_python}" -c 'import rclpy' >/dev/null 2>&1; then
  echo "error: ${system_python} cannot import ROS 2 Jazzy rclpy" >&2
  exit 1
fi

if ! "${system_python}" -c 'import sys; raise SystemExit(sys.version_info[:2] != (3, 12))'; then
  echo "error: inspection_rerun requires system CPython 3.12" >&2
  exit 1
fi

if [[ ! -x "${venv_dir}/bin/python" ]]; then
  "${system_python}" -m venv --system-site-packages "${venv_dir}"
fi

if ! "${venv_dir}/bin/python" -c 'import sys; raise SystemExit(sys.version_info[:2] != (3, 12))'; then
  echo "error: ${venv_dir} was not created with CPython 3.12; move it aside and rerun this script" >&2
  exit 1
fi

"${venv_dir}/bin/python" -m pip install --upgrade -r "${workspace_dir}/requirements-rerun.txt"
"${venv_dir}/bin/python" -c \
  'import cv2, numpy, rclpy, rerun, yaml; assert numpy.__version__ == "2.2.6"; assert rerun.__version__ == "0.36.2"; print("python 3.12"); print("rerun", rerun.__version__); print("rclpy", rclpy.__file__); print("numpy", numpy.__version__); print("opencv", cv2.__version__); print("yaml", yaml.__version__)'

echo "Setup complete. Activate with:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${venv_dir}/bin/activate"
