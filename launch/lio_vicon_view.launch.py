"""Launch selected LIO methods, live Vicon-aligned paths, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import SetRemap


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def generate_launch_description():
    point_launch = os.path.join(
        get_package_share_directory("point_lio"), "launch", "mapping_g1_mid360.launch.py"
    )
    spark_launch = os.path.join(
        get_package_share_directory("spark_fast_lio"), "launch", "mapping_g1_mid360.launch.yaml"
    )
    rviz_config = os.path.join(REPO_ROOT, "config", "lio_vicon_view.rviz")
    path_node = os.path.join(REPO_ROOT, "tools", "lio_vicon_path_publisher.py")

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
            cmd=["python3", path_node, "--save-dir", LaunchConfiguration("save_dir")],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["rviz2", "-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_viewer")),
        ),
    ])
