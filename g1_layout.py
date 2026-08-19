"""The G1 + RH56E2 DDS wire layout: fixed index/field order, not physics.

Single source of truth for anything that needs to agree with real hardware
on "index i means this joint" -- currently sim/mujoco/{body,hand}_bridge.py
(producing this layout from MuJoCo state) and viz/rerun/viz.py
(consuming it to drive URDF joints), so both sides can never drift apart on
the mapping itself. No dependency on mujoco, unitree_sdk2py, or anything
else -- plain data, importable from a raw-DDS-only environment too.
"""

# unitree_hg LowState_.motor_state / LowCmd_.motor_cmd index order, 0-28.
# See g1_joint_index_dds.md (unitree_mujoco) and g1_core_0.mcap (real
# hardware capture, mode_pr=0/serial confirmed).
BODY_JOINTS = [
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
]
assert len(BODY_JOINTS) == 29

MODE_PR = 0  # serial (pitch/roll direct); confirmed from g1_core_0.mcap /lowstate
MODE_MACHINE = 5  # confirmed from the same capture

# inspire_dds inspire_hand_state/inspire_hand_ctrl 6-element array order,
# confirmed from dds_close_fingers.py's explicit angle_set mapping, paired
# with each finger's joint name suffix (rh56e2_{left,right}.xml / the
# g1_29dof_rh56e2.xml <attach> naming: {l,r}_rh_{left,right}_{suffix}).
HAND_JOINTS = [
    ("pinky", "pinky_joint"),
    ("ring", "ring_joint"),
    ("middle", "middle_joint"),
    ("index", "index_joint"),
    ("thumb_flex", "thumb_joint2"),
    ("thumb_rot", "thumb_joint1"),
]

# MJCF <equality><joint> polycoef couplings (dependent = coef * driver),
# ported from inspire_hand_to_jointstate.py -- URDF has no equality-
# constraint concept, so anything driving URDF joints directly (viz) needs
# to apply these itself to move the coupled DIP/thumb3/4 joints too.
HAND_COUPLED_JOINTS = [
    ("thumb_joint3", "thumb_joint2", 0.8392),
    ("thumb_joint4", "thumb_joint3", 0.891),
    ("index_dip", "index_joint", 1.0843),
    ("middle_dip", "middle_joint", 1.0843),
    ("ring_dip", "ring_joint", 1.0843),
    ("pinky_dip", "pinky_joint", 1.0843),
]

HAND_SIDES = {"l": "left", "r": "right"}

# Tactile pad layout: (site_name, DDS var, rows, cols, axis_order). Shapes
# verified against the real RH56DFTP manual's own register map (section
# 2.6.20: tip=3x3, nail=12x8, pad=10x8, thumb middle=3x3, thumb pad=12x8,
# palm=8x14) -- DDS var names use different English words ("top"/"palm")
# for the same shapes the manual calls "nail"/"pad", not a structural
# mismatch.
#
# Site-to-pad assignment (which physical site gets which shape) is VERIFIED
# against real hardware two ways: (1) the manual's own hand diagram (section
# 2.6.20) shows, base-to-tip per finger, pad -> nail -> tip (thumb has an
# extra "middle section" between pad and nail); (2) rh56e2_left.xml's own
# body nesting matches that exactly -- force_sensor_1 sits alone on each
# finger's PROXIMAL body, while force_sensor_2 and _3 both sit one level
# deeper on the DISTAL body (confirmed for all 4 plain fingers via XML
# indent depth; thumb has 3 nesting levels for its 4 sensors, in the same
# base-to-tip order). So each plain finger's force_sensor_N is numbered
# base-to-tip: 1=pad (proximal), 2=nail (distal), 3=tip (distal, same body
# as _2) -- an earlier version of this table had 1 and 2 swapped (assigned
# by pad SIZE only, tip=3x3/nail=12x8/pad=10x8, without checking which
# physical segment each site sits on). Same fix applies to the thumb's
# sensor_1 (base, "pad") vs sensor_3 (distal, "nail") -- see
# sim/mujoco/hand_bridge.py.
#
# axis_order = (cols_axis, rows_axis, depth_axis), indices into each site's
# own size="sx sy sz" attribute. (0, 1, 2) for every pad except:
#   - palm_force_sensor: (1, 0, 2), both hands (same size="0.024995
#     0.015037 0.004" on left and right, so this is a structural asset
#     property, not a per-side quirk). (0, 1, 2) would put 8 cells across
#     the 50mm axis and 14 across the 30mm axis -> 6.25mm x 2.15mm cells, a
#     2.9:1 aspect nothing like a real taxel array; (1, 0, 2) gives 3.76mm x
#     3.57mm, a 1.05:1 aspect that actually looks like a uniform tactile
#     grid. Found by computing implied physical cell size under both
#     assignments for every pad in this table -- every OTHER pad's (0, 1,
#     2) was already the near-square (correct) choice, palm was the one
#     clear outlier.
#   - index_force_sensor_2 (left hand only): that one site's original asset
#     authoring has its thin/depth value in the size[0] slot instead of
#     size[2] like all 33 others (confirmed: every other site's size[2] is
#     exactly 0.004, that one's isn't, and its size[0] is) -- (2, 1, 0) tiles
#     across its true (Z, Y) surface plane instead of through the pad's depth.
#     This one is a HAND_TOUCH_AXIS_ORDER_OVERRIDE entry below (keyed to the
#     SITE, independent of which var it feeds), not a default-table change,
#     since it's a single-site asset defect, not a structural property.
HAND_TOUCH_PADS = [
    ("palm_force_sensor", "palm_touch", 14, 8, (1, 0, 2)),
    (
        "thumb_force_sensor_1",
        "fingerfive_palm_touch",
        12,
        8,
        (0, 1, 2),
    ),  # base segment = "pad"
    ("thumb_force_sensor_2", "fingerfive_middle_touch", 3, 3, (0, 1, 2)),
    (
        "thumb_force_sensor_3",
        "fingerfive_top_touch",
        12,
        8,
        (0, 1, 2),
    ),  # distal segment = "nail"
    ("thumb_force_sensor_4", "fingerfive_tip_touch", 3, 3, (0, 1, 2)),
    (
        "index_force_sensor_1",
        "fingerfour_palm_touch",
        10,
        8,
        (0, 1, 2),
    ),  # proximal body = "pad"
    (
        "index_force_sensor_2",
        "fingerfour_top_touch",
        12,
        8,
        (0, 1, 2),
    ),  # distal body = "nail"; (2,1,0) axis override on left hand only, see hand_bridge.py
    (
        "index_force_sensor_3",
        "fingerfour_tip_touch",
        3,
        3,
        (0, 1, 2),
    ),  # distal body = "tip"
    ("middle_force_sensor_1", "fingerthree_palm_touch", 10, 8, (0, 1, 2)),
    ("middle_force_sensor_2", "fingerthree_top_touch", 12, 8, (0, 1, 2)),
    ("middle_force_sensor_3", "fingerthree_tip_touch", 3, 3, (0, 1, 2)),
    ("ring_force_sensor_1", "fingertwo_palm_touch", 10, 8, (0, 1, 2)),
    ("ring_force_sensor_2", "fingertwo_top_touch", 12, 8, (0, 1, 2)),
    ("ring_force_sensor_3", "fingertwo_tip_touch", 3, 3, (0, 1, 2)),
    ("pinky_force_sensor_1", "fingerone_palm_touch", 10, 8, (0, 1, 2)),
    ("pinky_force_sensor_2", "fingerone_top_touch", 12, 8, (0, 1, 2)),
    ("pinky_force_sensor_3", "fingerone_tip_touch", 3, 3, (0, 1, 2)),
]
# The one exception, left hand only (see comment above and above table).
HAND_TOUCH_AXIS_ORDER_OVERRIDE = {("l", "index_force_sensor_2"): (2, 1, 0)}

# Pelvis world pose, geometry_msgs/PoseStamped (already vendored in
# unitree_sdk2py -- no custom IDL needed). Real LowState_ carries no world
# position, only joint angles + IMU orientation -- absolute pose is exactly
# what an external tracker provides (see hardware/vicon_bridge.py, which
# republishes motion_capture_tracking's world->g1_pelvis TF here). Sim
# publishes its own MuJoCo ground truth on the same topic as a stand-in
# (sim/mujoco/body_bridge.py), so viz consumes one signal either way.
VICON_PELVIS_TOPIC = "rt/vicon/pelvis"

# Vicon-pelvis calibration (VICON_OFFSET/IMU_IN_PELVIS) is applied once,
# upstream, in ros2/mocap/vicon_bridge.py -- rt/vicon/pelvis (bridged into
# this workspace's own /vicon/pelvis by bridge/unitree_ros2_bridge) already
# carries an already-calibrated pelvis pose, so this workspace doesn't need
# to know either constant exists. See that file's own docstring.

# Mid-360 LiDAR point cloud, sensor_msgs/PointCloud2 (already vendored in
# unitree_sdk2py). Confirmed against real hardware (2026-08-10, see
# tools/dds_recorder.py) at ~9.5Hz, 6 fields (x, y, z, intensity, ring,
# time), point_step=22 -- NOT the naive 4xfloat32/16-byte layout a first
# guess would assume, so any decoder needs to read field offsets from the
# message itself rather than hardcoding them (see tools/dense_map_mcap.py's
# own cloud_array for the same pattern).
LIDAR_TOPIC = "rt/utlidar/cloud_livox_mid360"

# Mid-360's own onboard IMU (sensor_msgs/Imu), separate from both
# rt/lowstate's pelvis IMU and rt/secondary_imu's torso one. Found by live
# DCPSPublication discovery against real hardware (2026-08-10, domain 0 --
# not documented anywhere in this SDK checkout or its examples). Message
# type isn't vendored in unitree_sdk2py at all (only PointCloud2_/PointField_
# exist under its sensor_msgs package) -- see sensor_msgs_imu.py for the
# hand-written IDL struct and why it lives outside the submodule. Orientation
# reads all-zero and every covariance field reads all-zero on this unit
# (confirmed real hardware, not a decode bug -- frame_id/timestamp/gyro/accel
# all decode to sane, stable values); only angular_velocity/linear_acceleration
# carry real data, and linear_acceleration is in g's, not m/s^2 (~1.0 magnitude
# at rest, not ~9.8).
LIDAR_IMU_TOPIC = "rt/utlidar/imu_livox_mid360"

# Commanded body-frame velocity (vx, vy, wz), geometry_msgs/Twist (linear.x/
# y, angular.z; already vendored in unitree_sdk2py). Published by whatever's
# driving an RL locomotion policy's command input (see
# rl/g1_locomotion_policy.py), consumed by viz/rerun/viz.py to draw it as an
# arrow anchored to the pelvis -- body-frame on purpose, so it can be logged
# straight under the "vicon/pelvis" entity and inherit that transform
# instead of needing a separate world-frame rotation.
CMD_VEL_TOPIC = "rt/cmd_vel"


def angle_act_to_rad(angle_act, joint_max):
    """angle_act 1000 = fully open (0 rad), 0 = fully closed (joint_max rad).
    Unverified against real hardware -- flip the sign if the real hand
    curls the wrong way (see inspire_hand_to_jointstate.py's own caveat)."""
    angle_act = max(0, min(1000, angle_act))
    return (1000 - angle_act) / 1000.0 * joint_max


def rad_to_angle_act(rad, joint_max):
    rad = max(0.0, min(joint_max, rad))
    return int(round(1000 - rad / joint_max * 1000))
