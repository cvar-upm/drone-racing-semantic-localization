"""Full-pipeline experiment launch for drone-racing semantic localization.

Brings up the dual_pose_graph SLAM node (from the ros-humble-dual-pose-graph
package), replays a recorded rosbag with the simulation clock, and optionally
opens RViz. The launch shuts itself down when bag playback finishes.

The SLAM node writes its CSV logs (slam_*.csv) to the working directory; after a
run, convert them to TUM and evaluate with evo (scripts/csv_to_tum.py and
scripts/run_evo.sh).

NOTE: not yet run end-to-end — the ros-humble-dual-pose-graph package must be
published/installed first. Topic and namespace assumptions follow config.yaml
(absolute input topics /ov_msckf/odomimu and /detections/gates are not affected
by the namespace).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory("dual_pose_graph")
    default_config = os.path.join(slam_share, "config", "config.yaml")
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_rviz_config = os.path.join(repo_root, "config", "config.rviz")
    qos_overrides = os.path.join(repo_root, "config", "qos_overrides.yaml")

    bag = LaunchConfiguration("bag")
    namespace = LaunchConfiguration("namespace")
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rate = LaunchConfiguration("rate")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    declared_args = [
        DeclareLaunchArgument("bag", description="Path to the rosbag to replay."),
        DeclareLaunchArgument(
            "namespace", default_value="drone", description="Drone namespace."),
        DeclareLaunchArgument(
            "config_file", default_value=default_config,
            description="SLAM node config (defaults to the package config.yaml)."),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the bag's recorded /clock as sim time."),
        DeclareLaunchArgument("rate", default_value="1.0", description="Bag playback rate."),
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz_config,
            description="RViz config file (defaults to config/config.rviz)."),
    ]

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "dual_pose_graph.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "config_file": config_file,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", "-s", "mcap", bag,
             "--qos-profile-overrides-path", qos_overrides,
             "--rate", rate],
        output="screen",
    )

    # End the whole launch once the bag has finished playing.
    shutdown_on_bag_end = RegisterEventHandler(
        OnProcessExit(target_action=bag_play, on_exit=[EmitEvent(event=Shutdown())]),
    )

    return LaunchDescription(
        declared_args + [slam_node, rviz_node, bag_play, shutdown_on_bag_end])
