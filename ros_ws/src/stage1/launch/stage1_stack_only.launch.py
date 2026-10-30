import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stage1_share = get_package_share_directory("stage1_stoneware_sim")
    world_file = os.path.join(
        stage1_share, "worlds", "stage1_raw_stoneware.world.sdf"
    )
    resource_paths = [
        os.path.join(stage1_share, "models"),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]

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
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
                ),
                launch_arguments={
                    "gz_args": IfElseSubstitution(
                        LaunchConfiguration("gazebo_gui"),
                        if_value=["-r -v 4 ", world_file],
                        else_value=["-s -r -v 4 ", world_file],
                    )
                }.items(),
            ),
        ]
    )
