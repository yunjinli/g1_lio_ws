# G1 Mid-360 online LIO workspace

Run Point-LIO and Spark FAST-LIO online from a Unitree G1 Mid-360 stream,
compare both LiDAR trajectories against Vicon in one world frame, display both
registered maps in RViz, save final world-frame PCD maps, and evaluate rigid
SVD-aligned ATE with a BEV overlay.

The Unitree SDK remains the hardware-facing transport. A small bridge converts
already received raw SDK messages into local ROS 2 topics; ROS 2 does not talk
to the robot directly.

## Repository layout

- `src/`: pinned upstream Point-LIO and Spark FAST-LIO submodules.
- `vendor/unitree_sdk2_python`: pinned Unitree SDK2 Python submodule.
- `overlays/`: G1 Mid-360 configurations and launch files copied into the LIO packages before building.
- `bridge/`: Unitree SDK to ROS 2 bridge implementation.
- `launch/lio_vicon_view.launch.py`: both LIOs, shared-world visualization, map accumulation, and RViz.
- `tools/lio_vicon_path_publisher.py`: pelvis-to-LiDAR kinematics, trajectory alignment, visualization topics, and PCD saving.
- `evaluation/evaluate_ate.py`: no-scale Umeyama/SVD ATE and BEV plot.

## Supported environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- `rmw_cyclonedds_cpp`

## Clone

```bash
git clone --recursive https://github.com/yunjinli/g1_lio_ws.git
cd g1_lio_ws
```

If already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Install dependencies

Install ROS 2 Jazzy first, then:

```bash
./scripts/install_dependencies_ubuntu.sh
```

Unitree SDK2 Python uses Cyclone DDS Python 0.10.x. Build the matching native
Cyclone DDS once:

```bash
git clone --branch releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git ~/cyclonedds
cmake -S ~/cyclonedds -B ~/cyclonedds/build \
  -DCMAKE_INSTALL_PREFIX=~/cyclonedds/install \
  -DCMAKE_BUILD_TYPE=Release
cmake --build ~/cyclonedds/build --target install -j"$(nproc)"
```

Create a Python environment that can also see the system ROS 2 Python modules:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
python -m pip install --upgrade pip
python -m pip install -e vendor/unitree_sdk2_python
```

## Build the LIO packages

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

`build.sh` applies the G1 overlays before running `colcon build`.

## Runtime environment

This project uses local ROS domain 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate

export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
```

The SDK-facing environment variables are owned by the application/setup that
already communicates with the G1. The bridge does not choose them. The
standalone runner used below reads the established contract:

```bash
export G1_DOMAIN_ID=0
export G1_NETWORK_INTERFACE=<robot-facing-interface>
```

For raw DDS replay on loopback, use `G1_DOMAIN_ID=1` and
`G1_NETWORK_INTERFACE=lo` instead.

## Run online

Terminal 1, bridge:

```bash
python tools/run_unitree_ros2_bridge.py
```

The SDK and ROS halves run in separate local processes joined by an
authenticated Unix socket. This is necessary when both use domain 1 because
Cyclone DDS rejects two configurations of the same domain in one process. The
socket is local IPC only; it performs no robot networking.

Terminal 2, both LIOs and RViz:

```bash
ros2 launch "$PWD/launch/lio_vicon_view.launch.py"
```

The view contains:

- Vicon-derived LiDAR trajectory (green).
- Point-LIO LiDAR trajectory and intensity-colored map.
- Spark FAST-LIO LiDAR trajectory and intensity-colored map.
- Current LiDAR coordinate axes for all three estimates.

Vicon is converted to the same physical `livox_frame` using the calibrated
marker-to-pelvis rotation, live waist yaw/roll/pitch from `/joint_states`, the
G1 waist/torso kinematic chain, and the official Mid-360 mount transform.

## Replay a recorded raw-DDS capture

Terminal 1 and 2 remain the same, except configure the bridge for loopback:

```bash
export G1_DOMAIN_ID=1 G1_NETWORK_INTERFACE=lo
python tools/run_unitree_ros2_bridge.py
```

Terminal 3:

```bash
export G1_DOMAIN_ID=1 G1_NETWORK_INTERFACE=lo
python tools/dds_replay_lio.py \
  --in ~/Downloads/recordings/walking_2 \
  --topics lidar,lidar_imu,lowstate,vicon_pelvis
```

## Save trajectories

Record incremental poses, not `/viz/*_path` (the latter repeatedly contains
the entire history and grows quadratically):

```bash
mkdir -p results
ros2 bag record -s mcap -o results/run_01 \
  /viz/vicon_lidar_pose \
  /viz/point_lio_lidar_pose \
  /viz/spark_fast_lio_lidar_pose \
  /viz/point_lio_map \
  /viz/spark_fast_lio_map \
  /vicon/pelvis /joint_states /aft_mapped_to_init /odometry
```

Stop recording with Ctrl+C.

## Save final maps

The comparison launch accumulates each registered scan, transforms it into
the same `world` frame, and writes binary PCD files when the launch exits
cleanly with Ctrl+C:

```text
results/maps/point_lio_map_world.pcd
results/maps/spark_fast_lio_map_world.pcd
```

Select another directory with:

```bash
ros2 launch "$PWD/launch/lio_vicon_view.launch.py" save_dir:=/absolute/output/path
```

Inspect a saved map with:

```bash
pcl_viewer results/maps/point_lio_map_world.pcd
```

## Compute SVD-aligned ATE and BEV overlay

Run this against the trajectory bag after the experiment:

```bash
python evaluation/evaluate_ate.py results/run_01 --out-dir results/run_01_eval
```

Outputs:

```text
results/run_01_eval/ate_metrics.csv
results/run_01_eval/trajectory_bev_svd.png
```

Alignment is rotation plus translation only—no scale—because LiDAR-inertial
odometry is metric. Samples are matched by bag receipt timestamp before the
least-squares SVD fit.

## Important interpretation

The online RViz alignment is fixed from the first common Vicon-derived LiDAR
and LIO pose. The offline ATE command independently computes the final global
least-squares rigid alignment over all time-matched positions. Neither method
changes scale or hides metric scale error.

## Render three progressive BEV videos

The trajectory bag above contains world-frame registered scans, allowing the
two LIO reconstructions to build progressively in the video rather than
showing the completed map from the first frame:

```bash
python evaluation/render_bev_videos.py results/run_01 \
  --out-dir results/run_01_videos --fps 20 --speed 1
```

Outputs:

```text
results/run_01_videos/vicon_bev.mp4
results/run_01_videos/point_lio_bev.mp4
results/run_01_videos/spark_fast_lio_bev.mp4
```

The Vicon video contains its LiDAR-frame trajectory. Each LIO video contains
its trajectory plus the intensity-colored point-cloud reconstruction accumulated
up to that video frame. `--speed 2` renders a two-times-real-time summary.
