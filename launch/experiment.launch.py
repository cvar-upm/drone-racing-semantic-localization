"""Full-pipeline experiment launch for drone-racing semantic localization.

Brings up the dual_pose_graph SLAM node (from the ros-humble-dual-pose-graph
package), replays a recorded rosbag, and optionally opens RViz. The launch shuts
itself down when bag playback finishes.

Each run writes into a per-run output directory (default runs/<bag-name>, override
with run_dir:=...): the node's CSV logs (slam_*.csv) land there, and the config
file used is copied alongside them so the run is reproducible. Convert the CSVs to
TUM and evaluate with evo (scripts/csv_to_tum.py and scripts/run_evo.sh), or run
scripts/evaluate_slam.py.

Input topics /ov_msckf/odomimu and /detections/gates are absolute (not namespaced).
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def launch_setup(context, *args, **kwargs):
    bag = os.path.abspath(LaunchConfiguration("bag").perform(context))
    config_file = os.path.abspath(LaunchConfiguration("config_file").perform(context))
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    rate = LaunchConfiguration("rate").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    run_dir = LaunchConfiguration("run_dir").perform(context)
    qos_overrides = os.path.join(REPO_ROOT, "config", "qos_overrides.yaml")

    if not run_dir:
        run_dir = os.path.join(REPO_ROOT, "runs", os.path.basename(bag.rstrip("/")))
    run_dir = os.path.abspath(run_dir)
    os.makedirs(run_dir, exist_ok=True)
    shutil.copy(config_file, os.path.join(run_dir, os.path.basename(config_file)))

    slam_node = Node(
        package="dual_pose_graph",
        executable="dual_pose_graph_node",
        name="dual_pose_graph_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        cwd=run_dir,
        parameters=[{"use_sim_time": use_sim_time}, config_file],
    )

    actions = [slam_node]

    if rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ))

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", "-s", "mcap", bag,
             "--qos-profile-overrides-path", qos_overrides,
             "--rate", rate],
        output="screen",
    )
    actions.append(bag_play)

    # End the whole launch once the bag has finished playing.
    actions.append(RegisterEventHandler(
        OnProcessExit(target_action=bag_play, on_exit=[EmitEvent(event=Shutdown())])))

    return actions


def generate_launch_description():
    slam_share = get_package_share_directory("dual_pose_graph")
    default_config = os.path.join(slam_share, "config", "config.yaml")
    default_rviz_config = os.path.join(REPO_ROOT, "config", "config.rviz")

    return LaunchDescription([
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
        DeclareLaunchArgument(
            "run_dir", default_value="",
            description="Output dir for CSVs + config copy (default runs/<bag-name>)."),
        OpaqueFunction(function=launch_setup),
    ])
