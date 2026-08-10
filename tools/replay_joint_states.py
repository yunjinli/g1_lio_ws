#!/usr/bin/env python3
"""Replay recorded Unitree low-state samples as a paced ROS 2 JointState stream."""

import argparse
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from g1_layout import BODY_JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("lowstate_jsonl")
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()
    with open(args.lowstate_jsonl) as source:
        records = [json.loads(line) for line in source if line.strip()]
    if not records:
        raise SystemExit("low-state recording is empty")

    rclpy.init()
    node = rclpy.create_node("recorded_joint_state_replay")
    publisher = node.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
    time.sleep(1.0)
    source_start = records[0]["t"]
    wall_start = time.monotonic()
    try:
        for record in records:
            deadline = wall_start + (record["t"] - source_start) / args.rate
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            motors = record["motor_state"][: len(BODY_JOINTS)]
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = list(BODY_JOINTS)
            message.position = [float(motor["q"]) for motor in motors]
            message.velocity = [float(motor["dq"]) for motor in motors]
            message.effort = [float(motor["tau_est"]) for motor in motors]
            publisher.publish(message)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
