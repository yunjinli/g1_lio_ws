#!/usr/bin/env python3
"""Render three progressive BEV MP4s from a g1_lio_ws trajectory/map bag."""

import argparse
import os

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

POSE_TOPICS = {
    "/viz/vicon_lidar_pose": "vicon",
    "/viz/point_lio_lidar_pose": "point_lio",
    "/viz/spark_fast_lio_lidar_pose": "spark_fast_lio",
}
MAP_TOPICS = {
    "/viz/point_lio_map": "point_lio",
    "/viz/spark_fast_lio_map": "spark_fast_lio",
}


def read_bag(uri, points_per_scan):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    poses = {name: [] for name in POSE_TOPICS.values()}
    scans = {name: [] for name in MAP_TOPICS.values()}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        t = timestamp_ns * 1e-9
        if topic in POSE_TOPICS:
            msg = deserialize_message(serialized, PoseStamped)
            p = msg.pose.position
            poses[POSE_TOPICS[topic]].append((t, p.x, p.y))
        elif topic in MAP_TOPICS:
            msg = deserialize_message(serialized, PointCloud2)
            points = np.asarray(point_cloud2.read_points_numpy(msg, field_names=["x", "y", "intensity"], skip_nans=True))
            if points.dtype.names:
                points = np.column_stack([points[field] for field in ("x", "y", "intensity")])
            points = points.reshape(-1, 3)
            if len(points) > points_per_scan:
                step = max(1, len(points) // points_per_scan)
                points = points[::step][:points_per_scan]
            scans[MAP_TOPICS[topic]].append((t, points.astype(np.float32, copy=False)))
    return (
        {name: np.asarray(values, dtype=np.float64) for name, values in poses.items()},
        scans,
    )


def limits(poses, scans):
    xy = [values[:, 1:3] for values in poses.values() if len(values)]
    xy += [scan[:, :2] for method in scans.values() for _, scan in method if len(scan)]
    all_xy = np.concatenate(xy)
    low, high = np.percentile(all_xy, [1, 99], axis=0)
    center = 0.5 * (low + high)
    radius = max(1.0, 0.58 * float(np.max(high - low)))
    return center[0] - radius, center[0] + radius, center[1] - radius, center[1] + radius


def render(method, pose, scans, output, bounds, fps, speed, max_map_points):
    if len(pose) < 2:
        print(f"warning: no trajectory for {method}; skipping {output}")
        return
    start, end = pose[0, 0], pose[-1, 0]
    frame_times = np.arange(start, end + speed / fps, speed / fps)
    fig, ax = plt.subplots(figsize=(8, 8), facecolor="#111111")
    ax.set_facecolor("#111111")
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#555555", alpha=0.3)
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    colors = {"vicon": "#2ecc71", "point_lio": "#e74c3c", "spark_fast_lio": "#3498db"}
    title = {"vicon": "Vicon-derived LiDAR", "point_lio": "Point-LIO + reconstruction", "spark_fast_lio": "Spark FAST-LIO + reconstruction"}[method]
    trajectory, = ax.plot([], [], color=colors[method], linewidth=2.5, zorder=3)
    current = ax.scatter([], [], c=colors[method], s=45, zorder=4)
    cloud = ax.scatter([], [], c=[], cmap="viridis", s=0.35, alpha=0.35, edgecolors="none", zorder=1)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", color="white")
    ax.set_title(title, color="white")
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_color("#888888")

    scan_index = 0
    accumulated = []
    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=5000, metadata={"title": title})
    with writer.saving(fig, output, dpi=140):
        for frame_time in frame_times:
            count = np.searchsorted(pose[:, 0], frame_time, side="right")
            visible = pose[:count, 1:3]
            trajectory.set_data(visible[:, 0], visible[:, 1])
            if len(visible):
                current.set_offsets(visible[-1:])
            while scan_index < len(scans) and scans[scan_index][0] <= frame_time:
                accumulated.append(scans[scan_index][1])
                scan_index += 1
            if accumulated:
                points = np.concatenate(accumulated)
                if len(points) > max_map_points:
                    points = points[:: max(1, len(points) // max_map_points)][:max_map_points]
                cloud.set_offsets(points[:, :2])
                cloud.set_array(points[:, 2])
            time_text.set_text(f"t = {frame_time - start:5.1f} s")
            writer.grab_frame()
    plt.close(fig)
    print(f"wrote {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--out-dir", default="results/videos")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed")
    parser.add_argument("--points-per-scan", type=int, default=2500)
    parser.add_argument("--max-map-points", type=int, default=500000)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    poses, scans = read_bag(args.bag, args.points_per_scan)
    bounds = limits(poses, scans)
    for method in ("vicon", "point_lio", "spark_fast_lio"):
        render(
            method,
            poses[method],
            scans.get(method, []),
            os.path.join(args.out_dir, f"{method}_bev.mp4"),
            bounds,
            args.fps,
            args.speed,
            args.max_map_points,
        )


if __name__ == "__main__":
    main()
