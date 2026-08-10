"""ROS 2 QoS profiles used by the Unitree sensor bridge."""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    # Reliable writers match both Point-LIO's best-effort cloud reader and
    # Spark FAST-LIO's reliable cloud reader.
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

IMU_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    # Point-LIO creates its IMU reader with reliable/default QoS, whereas
    # Spark FAST-LIO uses SensorDataQoS. A reliable writer serves both.
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
