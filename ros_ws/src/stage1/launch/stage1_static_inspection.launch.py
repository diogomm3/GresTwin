"""Start the Stage 1 scene paused with a mechanically fixed UR5e for inspection."""

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_stationary_robot_description():
    module_path = Path(__file__).with_name("stage1_robot_description.py")
    spec = importlib.util.spec_from_file_location(
        "stage1_robot_description", module_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load Stage 1 robot helper from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build_stationary_robot_description()


def generate_launch_description():
    stage1_share = get_package_share_directory("stage1_stoneware_sim")
    world_file = os.path.join(
        stage1_share, "worlds", "stage1_raw_stoneware.world.sdf"
    )
    model_path = os.path.join(stage1_share, "models")
    resource_paths = [model_path, os.environ.get("GZ_SIM_RESOURCE_PATH", "")]
    description, locked_joints = _build_stationary_robot_description()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Show the paused Stage 1 scene in Gazebo.",
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.pathsep.join(path for path in resource_paths if path),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
                ),
                launch_arguments={
                    "gz_args": IfElseSubstitution(
                        LaunchConfiguration("gazebo_gui"),
                        if_value=["-v 4 ", world_file],
                        else_value=["-s -v 4 ", world_file],
                    )
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": description,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-string", description, "-name", "ur"],
                output="screen",
            ),
            LogInfo(
                msg=(
                    "Stage 1 inspection: simulation starts paused; UR5e has no "
                    "gz_ros2_control plugin and no controllers; fixed joints: "
                    f"{locked_joints}."
                )
            ),
        ]
    )
