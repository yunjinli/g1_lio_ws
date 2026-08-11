#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS's setup.bash tests unset variables ($AMENT_TRACE_SETUP_FILES) without a
# default, so `set -u` above aborts the sourcing partway through and leaves the
# ament environment half-configured. Relax nounset only around the source.
set +u
source /opt/ros/jazzy/setup.bash
set -u
"$repo_root/scripts/apply_overlays.sh"
cd "$repo_root"
# --base-paths src keeps colcon out of vendor/, where unitree_sdk2py is a plain
# pip-installable package (README: `pip install -e vendor/unitree_sdk2py`), not a
# colcon one -- its setup.py opens README.md by relative path and fails inside
# colcon's build directory, aborting the LIO packages along with it.
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build complete. Run: source $repo_root/install/setup.bash"
