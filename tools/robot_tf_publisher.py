#!/usr/bin/env python3
"""Adapt G1 joint names for robot_state_publisher and anchor pelvis to Vicon world."""

import argparse
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class RobotTfPublisher(Node):
    def __init__(self, urdf_path):
        super().__init__("g1_robot_tf_publisher")
        root = ET.parse(urdf_path).getroot()
        self.urdf_joints = tuple(
            joint.get("name")
            for joint in root.findall("joint")
            if joint.get("type", "fixed") != "fixed"
        )
        self.publisher = self.create_publisher(
            JointState, "/robot_joint_states", qos_profile_sensor_data
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_positions = None
        self.latest_pelvis_pose = None
        self.received_joint_states = False
        self.received_vicon_pose = False
        self.create_subscription(
            JointState, "/joint_states", self.on_joints, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, "/vicon/pelvis", self.on_pelvis, qos_profile_sensor_data
        )
        # Publish the complete robot tree from one clock.  The Vicon and robot
        # drivers may use unrelated source timestamps, which otherwise leaves
        # RViz unable to find a common time between world and the robot links.
        self.create_timer(1.0 / 60.0, self.publish_current_tree)
        self.create_timer(5.0, self.report_missing_inputs)

    def on_joints(self, source):
        values = {
            name if name.endswith("_joint") else f"{name}_joint": position
            for name, position in zip(source.name, source.position)
        }
        self.latest_positions = [
            float(values.get(name, 0.0)) for name in self.urdf_joints
        ]
        self.received_joint_states = True

    def on_pelvis(self, pose):
        self.latest_pelvis_pose = pose.pose
        self.received_vicon_pose = True

    def publish_current_tree(self):
        stamp = self.get_clock().now().to_msg()
        if self.latest_positions is not None:
            message = JointState()
            message.header.stamp = stamp
            message.name = list(self.urdf_joints)
            message.position = self.latest_positions
            self.publisher.publish(message)

        if self.latest_pelvis_pose is None:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "pelvis"
        transform.transform.translation.x = self.latest_pelvis_pose.position.x
        transform.transform.translation.y = self.latest_pelvis_pose.position.y
        transform.transform.translation.z = self.latest_pelvis_pose.position.z
        transform.transform.rotation = self.latest_pelvis_pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def report_missing_inputs(self):
        missing = []
        if not self.received_vicon_pose:
            missing.append("/vicon/pelvis")
        if not self.received_joint_states:
            missing.append("/joint_states")
        if missing:
            self.get_logger().warning(
                "Robot TF is incomplete; still waiting for " + ", ".join(missing)
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = RobotTfPublisher(args.urdf)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
