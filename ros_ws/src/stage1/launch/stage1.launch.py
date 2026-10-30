import atexit
import os
import tempfile
import xml.etree.ElementTree as ET

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
from launch.substitutions import LaunchConfiguration


def _create_robot_offset_world(world_file, pallet_offset_y):
    root = ET.parse(world_file).getroot()
    pallet_count = 0
    cylinder_count = 0

    for include in root.findall(".//include"):
        model_name = include.findtext("name", "")
        if model_name == "stage1_pallet":
            pallet_count += 1
        elif model_name.startswith("stoneware_l"):
            cylinder_count += 1
        else:
            continue

        pose = include.find("pose")
        if pose is None or pose.text is None:
            raise RuntimeError(f"Model {model_name} has no initial pose")
        pose_values = pose.text.split()
        if len(pose_values) != 6:
            raise RuntimeError(f"Model {model_name} has an invalid initial pose")
        pose_values[1] = f"{float(pose_values[1]) + pallet_offset_y:.12f}"
        pose.text = " ".join(pose_values)

    if pallet_count != 1 or cylinder_count != 36:
        raise RuntimeError(
            "Expected one pallet and 36 cylinders in the Stage 1 world; "
            f"found {pallet_count} pallet(s) and {cylinder_count} cylinder(s)"
        )

    temporary_world = tempfile.NamedTemporaryFile(
        prefix="stage1_robot_offset_",
        suffix=".world.sdf",
        delete=False,
    )
    temporary_world.close()
    ET.ElementTree(root).write(
        temporary_world.name,
        encoding="utf-8",
        xml_declaration=True,
    )
    atexit.register(lambda path=temporary_world.name: os.path.exists(path) and os.unlink(path))
    return temporary_world.name


def _launch_robot_scene(context, ur5e_launch, source_world):
    pallet_offset_y = float(
        LaunchConfiguration("pallet_offset_y").perform(context)
    )
    world_file = _create_robot_offset_world(source_world, pallet_offset_y)
    return [
        LogInfo(
            msg=(
                "Stage 1 full launch: shifted pallet and 36-cylinder pyramid "
                f"by {pallet_offset_y:.3f} m in Y; table and physics unchanged."
            )
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
            DeclareLaunchArgument(
                "pallet_offset_y",
                default_value="-0.40",
                description=(
                    "Y offset applied to the pallet and the full cylinder pyramid "
                    "in the full robot launch."
                ),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.pathsep.join(path for path in resource_paths if path),
            ),
            OpaqueFunction(
                function=_launch_robot_scene,
                args=[ur5e_launch, world_file],
            ),
        ]
    )
