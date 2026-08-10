#!/usr/bin/env python3
"""Build live RViz paths for Vicon, Point-LIO, and Spark FAST-LIO.

Vicon is published directly in ``world``. Each LIO trajectory is rigidly
aligned once, when both its first odometry pose and the first Vicon pose are
available. This is an initial-pose alignment for live viewing, not the final
Umeyama alignment used for ATE evaluation.
"""

import argparse
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import rclpy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from g1_layout import VICON_OFFSET


def _quat_tuple(q):
    return (float(q.x), float(q.y), float(q.z), float(q.w))


def _normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    return tuple(value / norm for value in q) if norm else (0.0, 0.0, 0.0, 1.0)


def _conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def _rotate(q, p):
    x, y, z, w = q
    px, py, pz = p
    # Expanded q * [p,0] * conjugate(q), without normalizing the vector.
    tx, ty, tz = 2.0 * (y * pz - z * py), 2.0 * (z * px - x * pz), 2.0 * (x * py - y * px)
    return (
        px + w * tx + y * tz - z * ty,
        py + w * ty + z * tx - x * tz,
        pz + w * tz + x * ty - y * tx,
    )


def _position(p):
    return (float(p.x), float(p.y), float(p.z))


def _axis_angle(axis, angle):
    half = 0.5 * angle
    scale = math.sin(half)
    return _normalize((axis[0] * scale, axis[1] * scale, axis[2] * scale, math.cos(half)))


def _compose(p_parent, q_parent, translation, rotation):
    rotated = _rotate(q_parent, translation)
    return (
        tuple(p_parent[i] + rotated[i] for i in range(3)),
        _multiply(q_parent, rotation),
    )


class TrajectoryViewer(Node):
    def __init__(self, max_poses, save_dir):
        super().__init__("lio_vicon_path_publisher")
        # Register `world` as a real TF root so RViz does not report that its
        # fixed frame is absent when this trajectory-only viewer is running.
        self._static_tf = StaticTransformBroadcaster(self)
        world_tf = TransformStamped()
        world_tf.header.stamp = self.get_clock().now().to_msg()
        world_tf.header.frame_id = "world"
        world_tf.child_frame_id = "trajectory_view"
        world_tf.transform.rotation.w = 1.0
        self._static_tf.sendTransform(world_tf)
        self.max_poses = max_poses
        self.save_dir = os.path.abspath(os.path.expanduser(save_dir)) if save_dir else None
        self.map_points = {"point_lio": [], "spark_fast_lio": []}
        self.vicon_origin = None
        self.joints = {}
        self.alignments = {"point_lio": None, "spark_fast_lio": None}
        self.paths = {name: Path() for name in ("vicon", "point_lio", "spark_fast_lio")}
        self._path_publishers = {
            name: self.create_publisher(Path, f"/viz/{name}_path", 10)
            for name in self.paths
        }
        self._pose_publishers = {
            "vicon": self.create_publisher(PoseStamped, "/viz/vicon_lidar_pose", 10),
            "point_lio": self.create_publisher(PoseStamped, "/viz/point_lio_lidar_pose", 10),
            "spark_fast_lio": self.create_publisher(PoseStamped, "/viz/spark_fast_lio_lidar_pose", 10),
        }
        self._map_publishers = {
            "point_lio": self.create_publisher(PointCloud2, "/viz/point_lio_map", 10),
            "spark_fast_lio": self.create_publisher(PointCloud2, "/viz/spark_fast_lio_map", 10),
        }
        self.create_subscription(PoseStamped, "/vicon/pelvis", self._on_vicon, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/aft_mapped_to_init", lambda msg: self._on_lio("point_lio", msg), qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odometry", lambda msg: self._on_lio("spark_fast_lio", msg), qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/point_lio/cloud_registered",
            lambda msg: self._on_map("point_lio", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/spark_fast_lio/cloud_registered",
            lambda msg: self._on_map("spark_fast_lio", msg),
            qos_profile_sensor_data,
        )

    def _append(self, name, stamp, pose):
        stamped = PoseStamped()
        stamped.header.stamp = stamp
        stamped.header.frame_id = "world"
        stamped.pose = pose
        path = self.paths[name]
        path.header = stamped.header
        path.poses.append(stamped)
        if self.max_poses and len(path.poses) > self.max_poses:
            del path.poses[: len(path.poses) - self.max_poses]
        self._path_publishers[name].publish(path)
        self._pose_publishers[name].publish(stamped)

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def _vicon_lidar_pose(self, msg):
        """Convert raw Vicon marker pose through pelvis/waist/torso to LiDAR."""
        p = _position(msg.pose.position)
        q = _normalize(_quat_tuple(msg.pose.orientation))
        if VICON_OFFSET is not None:
            q = _multiply(q, VICON_OFFSET["rotation_xyzw"])
            offset = VICON_OFFSET["translation"]
            p = tuple(p[i] + _rotate(q, offset)[i] for i in range(3))

        # URDF chain: pelvis -> waist_yaw -> waist_roll -> torso_link.
        p, q = _compose(p, q, (0.0, 0.0, 0.0), _axis_angle((0.0, 0.0, 1.0), self.joints.get("waist_yaw", 0.0)))
        p, q = _compose(p, q, (-0.0039635, 0.0, 0.035), _axis_angle((1.0, 0.0, 0.0), self.joints.get("waist_roll", 0.0)))
        p, q = _compose(p, q, (0.0, 0.0, 0.019), _axis_angle((0.0, 1.0, 0.0), self.joints.get("waist_pitch", 0.0)))

        # Official G1 mount: torso -> mid360_link -> livox_frame.
        p, q = _compose(p, q, (0.0002835, 0.00003, 0.41618), _axis_angle((0.0, 1.0, 0.0), 0.04014257279586953))
        p, q = _compose(p, q, (0.0, 0.0, 0.0), _axis_angle((1.0, 0.0, 0.0), 3.141561))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = p
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def _on_vicon(self, msg):
        if not {"waist_yaw", "waist_roll", "waist_pitch"}.issubset(self.joints):
            return
        lidar_pose = self._vicon_lidar_pose(msg)
        if self.vicon_origin is None:
            self.vicon_origin = (_position(lidar_pose.position), _normalize(_quat_tuple(lidar_pose.orientation)))
            self.get_logger().info("received Vicon-derived LiDAR origin; live LIO alignment enabled")
        self._append("vicon", msg.header.stamp, lidar_pose)

    def _on_lio(self, name, msg):
        if self.vicon_origin is None:
            return
        pose = msg.pose.pose
        p_lio = _position(pose.position)
        q_lio = _normalize(_quat_tuple(pose.orientation))
        alignment = self.alignments[name]
        if alignment is None:
            p_vicon, q_vicon = self.vicon_origin
            q_align = _multiply(q_vicon, _conjugate(q_lio))
            rotated_origin = _rotate(q_align, p_lio)
            t_align = tuple(p_vicon[i] - rotated_origin[i] for i in range(3))
            alignment = (t_align, q_align)
            self.alignments[name] = alignment
            map_tf = TransformStamped()
            map_tf.header.stamp = self.get_clock().now().to_msg()
            map_tf.header.frame_id = "world"
            map_tf.child_frame_id = f"{name}_map"
            map_tf.transform.translation.x = t_align[0]
            map_tf.transform.translation.y = t_align[1]
            map_tf.transform.translation.z = t_align[2]
            map_tf.transform.rotation = Quaternion(
                x=q_align[0], y=q_align[1], z=q_align[2], w=q_align[3]
            )
            self._static_tf.sendTransform(map_tf)
            self.get_logger().info(f"fixed {name} initial alignment to Vicon")

        t_align, q_align = alignment
        rotated = _rotate(q_align, p_lio)
        transformed = Pose()
        transformed.position.x = rotated[0] + t_align[0]
        transformed.position.y = rotated[1] + t_align[1]
        transformed.position.z = rotated[2] + t_align[2]
        q_world = _multiply(q_align, q_lio)
        transformed.orientation = Quaternion(x=q_world[0], y=q_world[1], z=q_world[2], w=q_world[3])
        self._append(name, msg.header.stamp, transformed)

    def _on_map(self, name, msg):
        if self.alignments[name] is None:
            return
        points = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z", "intensity"], skip_nans=True
        )
        points = np.asarray(points)
        if points.dtype.names:
            points = np.column_stack([points[field] for field in ("x", "y", "z", "intensity")])
        points = points.reshape(-1, 4).astype(np.float32, copy=False)
        t_align, q_align = self.alignments[name]
        x, y, z, w = q_align
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)
        points[:, :3] = points[:, :3] @ rotation.T + np.asarray(t_align, dtype=np.float32)

        world_cloud = PointCloud2()
        world_cloud.header.stamp = msg.header.stamp
        world_cloud.header.frame_id = "world"
        world_cloud.height = 1
        world_cloud.width = len(points)
        world_cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        world_cloud.is_bigendian = False
        world_cloud.point_step = 16
        world_cloud.row_step = 16 * len(points)
        world_cloud.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        world_cloud.is_dense = True
        self._map_publishers[name].publish(world_cloud)
        if self.save_dir:
            self.map_points[name].append(points)

    def save_maps(self):
        if not self.save_dir:
            return
        os.makedirs(self.save_dir, exist_ok=True)
        for name, chunks in self.map_points.items():
            if not chunks:
                self.get_logger().warning(f"no {name} map scans received; no PCD written")
                continue
            points = np.concatenate(chunks)
            path = os.path.join(self.save_dir, f"{name}_map_world.pcd")
            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                "TYPE F F F F\nCOUNT 1 1 1 1\n"
                f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {len(points)}\nDATA binary\n"
            )
            with open(path, "wb") as output:
                output.write(header.encode("ascii"))
                output.write(np.ascontiguousarray(points, dtype=np.float32).tobytes())
            self.get_logger().info(f"saved {len(points)} world-frame points to {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-poses", type=int, default=20000, help="maximum poses retained per path; 0 is unlimited")
    parser.add_argument("--save-dir", default="", help="write both accumulated world-frame maps here on shutdown")
    args = parser.parse_args()
    rclpy.init()
    node = TrajectoryViewer(args.max_poses, args.save_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_maps()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
