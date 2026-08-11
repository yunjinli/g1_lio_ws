"""Launch selected LIO methods, live Vicon-aligned paths, and RViz."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _default_urdf():
    explicit = os.environ.get("G1_URDF")
    if explicit:
        return explicit

    controller_root = os.environ.get("G1_CONTROLLER_ROOT")
    candidates = []
    if controller_root:
        candidates.append(controller_root)
    # Support both layouts:
    #   lsy_g1_controller/g1_lio_ws (nested/self-contained)
    #   {lsy_g1_controller,g1_lio_ws} (sibling development checkouts)
    parent = os.path.dirname(REPO_ROOT)
    candidates.extend([parent, os.path.join(parent, "lsy_g1_controller")])
    for candidate in candidates:
        urdf = os.path.join(
            candidate, ".generated", "g1_29dof_rh56e2.urdf"
        )
        if os.path.isfile(urdf):
            return urdf

    # Keep the expected nested location in the launch argument so any error
    # describes a portable repository-relative path, not a developer's home.
    return os.path.join(parent, ".generated", "g1_29dof_rh56e2.urdf")


DEFAULT_URDF = _default_urdf()


def generate_launch_description():
    point_launch = os.path.join(
        get_package_share_directory("point_lio"), "launch", "mapping_g1_mid360.launch.py"
    )
    spark_launch = os.path.join(
        get_package_share_directory("spark_fast_lio"), "launch", "mapping_g1_mid360.launch.yaml"
    )
    rviz_config = os.path.join(REPO_ROOT, "config", "lio_vicon_view.rviz")
    path_node = os.path.join(REPO_ROOT, "tools", "lio_vicon_path_publisher.py")
    robot_tf_node = os.path.join(REPO_ROOT, "tools", "robot_tf_publisher.py")
    portable_urdf = os.path.join(REPO_ROOT, "tools", "portable_robot_description.py")
    asset_prefix = os.path.join(tempfile.gettempdir(), "g1_lio_ws_asset_prefix")
    rviz_environment = {
        "AMENT_PREFIX_PATH": asset_prefix
        + os.pathsep
        + os.environ.get("AMENT_PREFIX_PATH", "")
    }

    return LaunchDescription([
        DeclareLaunchArgument("start_viewer", default_value="true", description="Start comparison RViz"),
        DeclareLaunchArgument(
            "lio",
            default_value="all",
            choices=["all", "point-lio", "fast-lio"],
            description="LIO method to start",
        ),
        DeclareLaunchArgument(
            "save_dir",
            default_value=os.path.join(REPO_ROOT, "results", "maps"),
            description="Directory for final world-frame PCD maps written on shutdown",
        ),
        DeclareLaunchArgument(
            "urdf",
            default_value=DEFAULT_URDF,
            description="Robot URDF used for pelvis-to-mid360_link FK",
        ),
        GroupAction(condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("lio"), "' in ('all', 'point-lio')"
        ])), actions=[
            SetRemap(src="/cloud_registered", dst="/point_lio/cloud_registered"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(point_launch),
                launch_arguments={"rviz": "false"}.items(),
            ),
        ]),
        GroupAction(condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("lio"), "' in ('all', 'fast-lio')"
        ])), actions=[
            SetRemap(src="/cloud_registered", dst="/spark_fast_lio/cloud_registered"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(spark_launch),
                launch_arguments={"start_rviz": "false"}.items(),
            ),
        ]),
        ExecuteProcess(
            cmd=[
                "python3",
                path_node,
                "--save-dir",
                LaunchConfiguration("save_dir"),
                "--urdf",
                LaunchConfiguration("urdf"),
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                robot_tf_node,
                "--urdf",
                LaunchConfiguration("urdf"),
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    Command([
                        "python3 ",
                        portable_urdf,
                        " --urdf ",
                        LaunchConfiguration("urdf"),
                    ]),
                    value_type=str,
                )
            }],
            remappings=[("/joint_states", "/robot_joint_states")],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["rviz2", "-d", rviz_config],
            additional_env=rviz_environment,
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_viewer")),
        ),
    ])
