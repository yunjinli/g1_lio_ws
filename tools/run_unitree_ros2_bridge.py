#!/usr/bin/env python3
"""Standalone bootstrap for the Unitree-SDK-to-ROS-2 bridge.

The bridge component itself owns no SDK/network configuration. This runner is
only for operating it as a standalone SDK application (including alongside
tools/dds_replay.py), so it initializes the SDK using the same environment
contract as every existing repository tool: G1_DOMAIN_ID and
G1_NETWORK_INTERFACE. There are intentionally no bridge-specific networking
arguments or defaults.

Run through the ros2 Pixi environment after sourcing ROS 2, for example:

  G1_DOMAIN_ID=1 G1_NETWORK_INTERFACE=lo pixi run -e ros2 bash -lc \
    'source /opt/ros/jazzy/setup.bash && python tools/run_unitree_ros2_bridge.py'
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from bridge.unitree_ros2_bridge.ipc_sink import run as run_sink


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--direct", action="store_true", help="use one process; only valid when SDK and ROS use different DDS domains")
    args = parser.parse_args()

    domain_id = os.environ.get("G1_DOMAIN_ID")
    interface = os.environ.get("G1_NETWORK_INTERFACE")
    if domain_id is None or not interface:
        parser.error(
            "the standalone SDK application requires G1_DOMAIN_ID and "
            "G1_NETWORK_INTERFACE; select an existing Pixi environment or set them externally"
        )

    if args.direct:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from bridge.unitree_ros2_bridge import UnitreeRos2Bridge

        ChannelFactoryInitialize(int(domain_id), interface)
        rclpy.init()
        node = UnitreeRos2Bridge()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    socket_path = os.path.join(tempfile.gettempdir(), f"g1-ros2-bridge-{os.getpid()}.sock")
    source_code = (
        "from bridge.unitree_ros2_bridge.ipc_source import run; "
        f"run({socket_path!r})"
    )
    source = subprocess.Popen([sys.executable, "-c", source_code])
    try:
        deadline = time.monotonic() + 5.0
        while not os.path.exists(socket_path):
            if source.poll() is not None:
                raise RuntimeError("SDK source process exited before opening its local socket")
            if time.monotonic() >= deadline:
                raise RuntimeError("timed out waiting for SDK source process")
            time.sleep(0.05)
        run_sink(socket_path)
    finally:
        source.terminate()
        try:
            source.wait(timeout=3)
        except subprocess.TimeoutExpired:
            source.kill()
            source.wait()
        if os.path.exists(socket_path):
            os.unlink(socket_path)


if __name__ == "__main__":
    main()
