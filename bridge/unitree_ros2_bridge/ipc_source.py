"""SDK-only half of the bridge, used when SDK DDS and ROS share a domain."""

import os
import threading
from multiprocessing.connection import Listener

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from g1_layout import LIDAR_IMU_TOPIC, LIDAR_TOPIC, VICON_PELVIS_TOPIC
from sensor_msgs_imu import Imu_


def run(socket_path):
    domain_id = os.environ.get("G1_DOMAIN_ID")
    interface = os.environ.get("G1_NETWORK_INTERFACE")
    if domain_id is None or not interface:
        raise RuntimeError("G1_DOMAIN_ID and G1_NETWORK_INTERFACE must be configured externally")
    ChannelFactoryInitialize(int(domain_id), interface)

    # AF_UNIX is machine-local. Authentication prevents an unrelated local
    # process from injecting pickled objects into this trusted bridge channel.
    listener = Listener(socket_path, family="AF_UNIX", authkey=b"g1-ros2-bridge")
    print(f"[unitree_ros2_bridge/source] waiting for ROS sink at {socket_path}", flush=True)
    connection = listener.accept()
    print("[unitree_ros2_bridge/source] ROS sink connected", flush=True)
    send_lock = threading.Lock()

    def callback(key):
        def send(raw):
            with send_lock:
                connection.send((key, raw))
        return send

    subscribers = [
        ChannelSubscriber(LIDAR_TOPIC, PointCloud2_),
        ChannelSubscriber(LIDAR_IMU_TOPIC, Imu_),
        ChannelSubscriber("rt/lowstate", LowState_),
        ChannelSubscriber(VICON_PELVIS_TOPIC, PoseStamped_),
    ]
    for subscriber, key, depth in zip(
        subscribers, ("cloud", "imu", "joints", "vicon"), (4, 200, 20, 20)
    ):
        subscriber.Init(callback(key), depth)

    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()
        listener.close()
