"""Attach to an initialized Unitree SDK and republish selected data in ROS 2.

This component deliberately does not call ChannelFactoryInitialize and does
not choose a DDS domain or network interface. Its owning application must
initialize unitree_sdk2py before constructing UnitreeRos2Bridge.
"""

import threading

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from g1_layout import LIDAR_IMU_TOPIC, LIDAR_TOPIC, VICON_PELVIS_TOPIC
from sensor_msgs_imu import Imu_

from . import conversions
from .qos import IMU_QOS, SENSOR_QOS, STATE_QOS


class UnitreeRos2Bridge(Node):
    """Unitree SDK subscriber callbacks plus ordinary ROS 2 publishers."""

    def __init__(self, attach_sdk=True):
        super().__init__("unitree_ros2_bridge")
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/utlidar/cloud_livox_mid360", SENSOR_QOS
        )
        self.imu_pub = self.create_publisher(
            Imu, "/utlidar/imu_livox_mid360", IMU_QOS
        )
        self.joints_pub = self.create_publisher(JointState, "/joint_states", STATE_QOS)
        self.vicon_pub = self.create_publisher(PoseStamped, "/vicon/pelvis", STATE_QOS)

        self._lock = threading.Lock()
        self._counts = {"cloud": 0, "imu": 0, "joints": 0, "vicon": 0}
        self._subscribers = []
        if attach_sdk:
            self._subscribers = [
                self._subscribe(LIDAR_TOPIC, PointCloud2_, self._on_cloud, 4),
                self._subscribe(LIDAR_IMU_TOPIC, Imu_, self._on_imu, 200),
                self._subscribe("rt/lowstate", LowState_, self._on_lowstate, 20),
                self._subscribe(VICON_PELVIS_TOPIC, PoseStamped_, self._on_vicon, 20),
            ]
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            ("attached to initialized Unitree SDK; " if attach_sdk else "IPC input active; ")
            + "publishing cloud, IMU, joints, and Vicon"
        )

    def publish_raw(self, key, raw):
        callbacks = {
            "cloud": self._on_cloud,
            "imu": self._on_imu,
            "joints": self._on_lowstate,
            "vicon": self._on_vicon,
        }
        callbacks[key](raw)

    @staticmethod
    def _subscribe(topic, msg_type, callback, queue_len):
        subscriber = ChannelSubscriber(topic, msg_type)
        subscriber.Init(callback, queue_len)
        return subscriber  # retain ownership for the lifetime of the node

    def _increment(self, key):
        with self._lock:
            self._counts[key] += 1

    def _on_cloud(self, raw):
        self.cloud_pub.publish(conversions.pointcloud2(raw))
        self._increment("cloud")

    def _on_imu(self, raw):
        self.imu_pub.publish(conversions.imu(raw))
        self._increment("imu")

    def _on_lowstate(self, raw):
        now = self.get_clock().now().to_msg()
        self.joints_pub.publish(conversions.joint_state(raw, now))
        self._increment("joints")

    def _on_vicon(self, raw):
        self.vicon_pub.publish(conversions.pose_stamped(raw))
        self._increment("vicon")

    def _report(self):
        with self._lock:
            counts = dict(self._counts)
        self.get_logger().info(
            "received totals: " + ", ".join(f"{key}={value}" for key, value in counts.items())
        )
