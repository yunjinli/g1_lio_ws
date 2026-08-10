"""ROS-only half of the bridge, receiving SDK messages over a Unix socket."""

import threading
from multiprocessing.connection import Client

import rclpy
from rclpy.executors import ExternalShutdownException

from .bridge import UnitreeRos2Bridge


def run(socket_path):
    connection = Client(socket_path, family="AF_UNIX", authkey=b"g1-ros2-bridge")
    rclpy.init()
    node = UnitreeRos2Bridge(attach_sdk=False)

    def receive():
        try:
            while rclpy.ok():
                key, raw = connection.recv()
                node.publish_raw(key, raw)
        except (EOFError, OSError):
            if rclpy.ok():
                rclpy.shutdown()

    thread = threading.Thread(target=receive, name="unitree_bridge_ipc", daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        connection.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
