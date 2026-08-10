#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

install -D "$repo_root/overlays/point_lio/config/g1_mid360.yaml" \
  "$repo_root/src/point_lio_ros2/config/g1_mid360.yaml"
install -D "$repo_root/overlays/point_lio/launch/mapping_g1_mid360.launch.py" \
  "$repo_root/src/point_lio_ros2/launch/mapping_g1_mid360.launch.py"
install -D "$repo_root/overlays/spark_fast_lio/config/g1_mid360.yaml" \
  "$repo_root/src/spark-fast-lio/spark_fast_lio/config/g1_mid360.yaml"
install -D "$repo_root/overlays/spark_fast_lio/launch/mapping_g1_mid360.launch.yaml" \
  "$repo_root/src/spark-fast-lio/spark_fast_lio/launch/mapping_g1_mid360.launch.yaml"

echo "Applied G1 Mid-360 overlays."
