#!/usr/bin/env python3
"""Compute rigid SVD-aligned ATE and save a BEV overlay from a ROS 2 bag.

The bag must contain the incremental shared-world LiDAR pose topics produced
by tools/lio_vicon_path_publisher.py. Bag receipt timestamps are used because
the Vicon bridge may not populate a comparable sensor header timestamp.
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

TOPICS = {
    "/viz/vicon_lidar_pose": "vicon",
    "/viz/point_lio_lidar_pose": "point_lio",
    "/viz/spark_fast_lio_lidar_pose": "spark_fast_lio",
}


def nearest_time_match(query_t, reference_t, max_dt):
    idx = np.searchsorted(reference_t, query_t)
    idx = np.clip(idx, 1, len(reference_t) - 1)
    left, right = idx - 1, idx
    nearest = np.where(
        np.abs(query_t - reference_t[left]) <= np.abs(reference_t[right] - query_t), left, right
    )
    keep = np.abs(query_t - reference_t[nearest]) <= max_dt
    return nearest[keep], keep


def umeyama_rigid(source, target):
    source_mean, target_mean = source.mean(0), target.mean(0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _, vt = np.linalg.svd(covariance)
    sign = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1.0, 1.0, sign]) @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def read_bag(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    streams = {name: [] for name in TOPICS.values()}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        msg = deserialize_message(serialized, PoseStamped)
        p = msg.pose.position
        streams[TOPICS[topic]].append((timestamp_ns * 1e-9, p.x, p.y, p.z))
    return {name: np.asarray(values, dtype=np.float64) for name, values in streams.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag2 directory recorded with storage id mcap")
    parser.add_argument("--out-dir", default="results")
    parser.add_argument("--max-dt", type=float, default=0.02)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    streams = read_bag(args.bag)
    if len(streams["vicon"]) < 2:
        raise SystemExit("bag has no usable /viz/vicon_lidar_pose stream")

    reference = streams["vicon"]
    reference = reference[np.argsort(reference[:, 0])]
    results = []
    aligned_for_plot = {}
    for name in ("point_lio", "spark_fast_lio"):
        estimate = streams[name]
        if len(estimate) < 2:
            print(f"warning: no usable {name} stream")
            continue
        estimate = estimate[np.argsort(estimate[:, 0])]
        nearest, keep = nearest_time_match(estimate[:, 0], reference[:, 0], args.max_dt)
        source, target = estimate[keep, 1:4], reference[nearest, 1:4]
        if len(source) < 3:
            print(f"warning: only {len(source)} time matches for {name}")
            continue
        rotation, translation = umeyama_rigid(source, target)
        aligned = source @ rotation.T + translation
        error = np.linalg.norm(aligned - target, axis=1)
        results.append({
            "method": name,
            "matched": len(error),
            "rmse_m": float(np.sqrt(np.mean(error**2))),
            "mean_m": float(error.mean()),
            "median_m": float(np.median(error)),
            "std_m": float(error.std()),
            "max_m": float(error.max()),
        })
        aligned_for_plot[name] = aligned

    metrics_path = os.path.join(args.out_dir, "ate_metrics.csv")
    with open(metrics_path, "w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=["method", "matched", "rmse_m", "mean_m", "median_m", "std_m", "max_m"])
        writer.writeheader()
        writer.writerows(results)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(reference[:, 1], reference[:, 2], color="black", linewidth=2, label="Vicon-derived LiDAR")
    colors = {"point_lio": "#d62728", "spark_fast_lio": "#1f77b4"}
    for row in results:
        name = row["method"]
        points = aligned_for_plot[name]
        ax.plot(points[:, 0], points[:, 1], color=colors[name], linewidth=1.5,
                label=f"{name} (ATE RMSE {100 * row['rmse_m']:.1f} cm)")
        print(f"{name}: matched={row['matched']} RMSE={row['rmse_m']:.4f} m mean={row['mean_m']:.4f} m max={row['max_m']:.4f} m")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.set_title("Rigid SVD-aligned LiDAR trajectories")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    bev_path = os.path.join(args.out_dir, "trajectory_bev_svd.png")
    fig.savefig(bev_path, dpi=180)
    print(f"wrote {metrics_path}\nwrote {bev_path}")


if __name__ == "__main__":
    main()
