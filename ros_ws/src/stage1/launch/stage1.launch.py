import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stage1_share = get_package_share_directory("stage1_stoneware_sim")
    ur5e_share = get_package_share_directory("ur5e_moveit_test")
    model_path = os.path.join(stage1_share, "models")
    resource_paths = [model_path, os.environ.get("GZ_SIM_RESOURCE_PATH", "")]

    ur5e_launch = os.path.join(
        ur5e_share, "launch", "ur5e_moveit.launch.py"
    )
    world_file = os.path.join(
        stage1_share, "worlds", "stage1_raw_stoneware.world.sdf"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo with its graphical interface.",
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.pathsep.join(path for path in resource_paths if path),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur5e_launch),
                launch_arguments={
                    "world_file": world_file,
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rviz_enabled": "false",
                    "run_pick_place": "false",
                    "moveit_enabled": "false",
                    "stage1_end_effector": "true",
                    "enable_detachable_joint": "false",
                }.items(),
            ),
        ]
    )
