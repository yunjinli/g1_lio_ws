"""sensor_msgs/msg/Imu, hand-written to match the standard ROS2 IDL layout --
NOT vendored anywhere in unitree_sdk2py (confirmed by grepping its whole
sensor_msgs package, which only ships PointCloud2_/PointField_), even though
the real G1 publishes it: `rt/utlidar/imu_livox_mid360` was found via a live
DCPSPublication discovery scan (cyclonedds.builtin.BuiltinDataReader against
a real robot, 2026-08-10) advertising type_name
"sensor_msgs::msg::dds_::Imu_" -- the Mid-360 LiDAR's own onboard IMU,
distinct from both rt/lowstate's pelvis IMU and rt/secondary_imu's torso one.

Kept as its own top-level module rather than added to the unitree_sdk2py
submodule checkout (core/unitree_sdk2_python) -- that submodule tracks
upstream unitreerobotics/unitree_sdk2_python verbatim, and a message type
missing from Unitree's own IDL set doesn't belong patched into their fork.
Field layout/order matches ROS2's rosidl-generated sensor_msgs/Imu.idl
exactly (same struct every other sensor_msgs/geometry_msgs/std_msgs type
here was generated from) -- CDR is positional, not self-describing, so this
has to match byte-for-byte or decoding silently produces garbage.
"""
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Imu_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Imu_"):
    header: Header_
    orientation: Quaternion_
    orientation_covariance: types.array[types.float64, 9]
    angular_velocity: Vector3_
    angular_velocity_covariance: types.array[types.float64, 9]
    linear_acceleration: Vector3_
    linear_acceleration_covariance: types.array[types.float64, 9]
