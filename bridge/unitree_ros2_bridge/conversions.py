"""Pure conversions from unitree_sdk2py IDL objects to ROS 2 messages."""

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
from std_msgs.msg import Header

from g1_layout import BODY_JOINTS


def ros_time(stamp) -> Time:
    return Time(sec=int(stamp.sec), nanosec=int(stamp.nanosec))


def header(raw) -> Header:
    return Header(stamp=ros_time(raw.stamp), frame_id=raw.frame_id)


def pointcloud2(raw) -> PointCloud2:
    """Copy the cloud envelope while retaining its packed point byte layout."""
    msg = PointCloud2()
    msg.header = header(raw.header)
    msg.height = int(raw.height)
    msg.width = int(raw.width)
    msg.fields = [
        PointField(
            name=field.name,
            offset=int(field.offset),
            datatype=int(field.datatype),
            count=int(field.count),
        )
        for field in raw.fields
    ]
    msg.is_bigendian = bool(raw.is_bigendian)
    msg.point_step = int(raw.point_step)
    msg.row_step = int(raw.row_step)
    # rclpy accepts bytes and copies them into the outgoing serialized sample.
    # This deliberately avoids unpacking/repacking tens of thousands of points.
    msg.data = bytes(raw.data)
    msg.is_dense = bool(raw.is_dense)
    return msg


def imu(raw) -> Imu:
    msg = Imu()
    msg.header = header(raw.header)
    msg.orientation = Quaternion(
        x=float(raw.orientation.x),
        y=float(raw.orientation.y),
        z=float(raw.orientation.z),
        w=float(raw.orientation.w),
    )
    msg.orientation_covariance = list(raw.orientation_covariance)
    msg.angular_velocity = Vector3(
        x=float(raw.angular_velocity.x),
        y=float(raw.angular_velocity.y),
        z=float(raw.angular_velocity.z),
    )
    msg.angular_velocity_covariance = list(raw.angular_velocity_covariance)
    msg.linear_acceleration = Vector3(
        x=float(raw.linear_acceleration.x),
        y=float(raw.linear_acceleration.y),
        z=float(raw.linear_acceleration.z),
    )
    msg.linear_acceleration_covariance = list(raw.linear_acceleration_covariance)
    return msg


def pose_stamped(raw) -> PoseStamped:
    msg = PoseStamped()
    msg.header = header(raw.header)
    msg.pose = Pose(
        position=Point(
            x=float(raw.pose.position.x),
            y=float(raw.pose.position.y),
            z=float(raw.pose.position.z),
        ),
        orientation=Quaternion(
            x=float(raw.pose.orientation.x),
            y=float(raw.pose.orientation.y),
            z=float(raw.pose.orientation.z),
            w=float(raw.pose.orientation.w),
        ),
    )
    return msg


def joint_state(raw, stamp: Time) -> JointState:
    """Publish the canonical 29-DoF G1 body order from rt/lowstate."""
    motors = raw.motor_state[: len(BODY_JOINTS)]
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = list(BODY_JOINTS)
    msg.position = [float(motor.q) for motor in motors]
    msg.velocity = [float(motor.dq) for motor in motors]
    msg.effort = [float(motor.tau_est) for motor in motors]
    return msg
