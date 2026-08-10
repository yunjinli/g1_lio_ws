#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: $0 RAW_RECORDING_DIR ROS_INPUT_BAG OUTPUT_DIR [VIDEO_SPEED]" >&2
  exit 2
fi

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
raw_dir=$(realpath "$1")
input_bag=$(realpath "$2")
output_dir=$(realpath -m "$3")
video_speed=${4:-2}

if [[ -e "$output_dir" ]]; then
  echo "refusing to overwrite existing output: $output_dir" >&2
  exit 1
fi
if [[ ! -f "$raw_dir/lowstate.jsonl" ]]; then
  echo "missing $raw_dir/lowstate.jsonl" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$repo_root/install/setup.bash"
export PYTHONPATH="$repo_root${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mkdir -p "$output_dir/maps" "$output_dir/logs"
launch_group=""
record_group=""
joints_pid=""

stop_group() {
  local signal=$1 pid=${2:-}
  [[ -n "$pid" ]] && kill -"$signal" -- "-$pid" 2>/dev/null || true
}

cleanup() {
  [[ -n "$joints_pid" ]] && kill -TERM "$joints_pid" 2>/dev/null || true
  stop_group TERM "$record_group"
  stop_group TERM "$launch_group"
}
trap cleanup EXIT INT TERM

setsid ros2 launch "$repo_root/launch/lio_vicon_view.launch.py" \
  start_viewer:=false save_dir:="$output_dir/maps" >"$output_dir/logs/launch.log" 2>&1 &
launch_group=$!
sleep 5

setsid ros2 bag record -s mcap -o "$output_dir/trajectory_map_bag" --topics \
  /viz/vicon_lidar_pose /viz/point_lio_lidar_pose /viz/spark_fast_lio_lidar_pose \
  /viz/point_lio_map /viz/spark_fast_lio_map >"$output_dir/logs/record.log" 2>&1 &
record_group=$!
sleep 2

python3 "$repo_root/tools/replay_joint_states.py" "$raw_dir/lowstate.jsonl" \
  >"$output_dir/logs/joints.log" 2>&1 &
joints_pid=$!
ros2 bag play "$input_bag"
wait "$joints_pid" || true
joints_pid=""
sleep 5

stop_group INT "$record_group"
sleep 3
stop_group TERM "$record_group"
wait "$record_group" || true
record_group=""

stop_group INT "$launch_group"
sleep 10
stop_group TERM "$launch_group"
wait "$launch_group" || true
launch_group=""

python3 "$repo_root/evaluation/evaluate_ate.py" "$output_dir/trajectory_map_bag" \
  --out-dir "$output_dir/evaluation" --max-dt 0.03
python3 "$repo_root/evaluation/render_bev_videos.py" "$output_dir/trajectory_map_bag" \
  --out-dir "$output_dir/videos" --fps 20 --speed "$video_speed" --max-map-points 150000

trap - EXIT
echo "completed: $output_dir"
