#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  git build-essential cmake ninja-build ffmpeg python3-pip python3-numpy python3-matplotlib \
  ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-pcl-ros \
  ros-jazzy-rosbag2-storage-mcap ros-jazzy-sensor-msgs-py \
  libpcl-dev libeigen3-dev

echo "System dependencies installed. See README.md for Cyclone DDS Python/Unitree SDK setup."
