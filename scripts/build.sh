#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
"$repo_root/scripts/apply_overlays.sh"
cd "$repo_root"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build complete. Run: source $repo_root/install/setup.bash"
