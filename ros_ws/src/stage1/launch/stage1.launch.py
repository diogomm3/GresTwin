import importlib.util
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _load_robot_description_builder():
    module_path = Path(__file__).with_name("stage1_robot_description.py")
    spec = importlib.util.spec_from_file_location(
        "stage1_robot_description", module_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load Stage 1 robot helper from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build_stage1_robot_description


def generate_launch_description():
    stage1_share = get_package_share_directory("stage1_stoneware_sim")
    world_file = os.path.join(
        stage1_share, "worlds", "stage1_raw_stoneware.world.sdf"
    )
    trajectory_controller_config = os.path.join(
        stage1_share, "config", "stage1_trajectory_controller.yaml"
    )
    scene_config = json.loads(
        Path(stage1_share, "config", "stage1_scene.json").read_text(
            encoding="utf-8"
        )
    )
    cylinder_level_counts = scene_config["cylinder"]["level_counts_bottom_up"]
    cylinder_names = [
        f"stoneware_l{level}_{index}"
        for level, count in enumerate(cylinder_level_counts, start=1)
        for index in range(1, count + 1)
    ]
    model_path = os.path.join(stage1_share, "models")
    resource_paths = [
        model_path,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]
    robot_description = _load_robot_description_builder()(
        scene_config["robot"]["base_pose"],
    )
    ur_description_share = get_package_share_directory("ur_description")
    ur_moveit_share = get_package_share_directory("ur_moveit_config")
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=os.path.join(ur_description_share, "urdf", "ur.urdf.xacro"),
            mappings={
                "name": "ur",
                "ur_type": "ur5e",
                "safety_limits": "true",
                "safety_pos_margin": "0.15",
                "safety_k_position": "20",
                "tf_prefix": "",
            },
        )
        .robot_description_semantic(
            file_path="srdf/ur.srdf.xacro",
            mappings={"name": "ur"},
        )
        .to_moveit_configs()
    )
    moveit_parameters = moveit_config.to_dict()
    moveit_parameters.update(
        {
            "robot_description": robot_description,
            "use_sim_time": True,
            "allow_trajectory_execution": ParameterValue(
                LaunchConfiguration("run_first_pick_place"),
                value_type=bool,
            ),
            "publish_robot_description_semantic": True,
        }
    )
    moveit_parameters.setdefault("trajectory_execution", {})[
        "allowed_start_tolerance"
    ] = 0.03
    moveit_parameters.setdefault("robot_description_kinematics", {}).setdefault(
        "ur_manipulator", {}
    )["kinematics_solver_timeout"] = 0.2

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    robot_description_republisher = Node(
        package="ur5e_moveit_test",
        executable="republish_robot_description.py",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "scaled_joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            trajectory_controller_config,
        ],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_parameters],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            "-d",
            os.path.join(ur_moveit_share, "config", "moveit.rviz"),
        ],
        parameters=[moveit_parameters],
        condition=IfCondition(LaunchConfiguration("rviz_enabled")),
    )
    planning_scene_publisher = Node(
        package="stage1_stoneware_sim",
        executable="publish_stage1_planning_scene.py",
        parameters=[
            {
                "world_file": world_file,
                "models_dir": model_path,
            }
        ],
        output="screen",
    )
    pallet = scene_config["pallet"]
    cylinder = scene_config["cylinder"]
    table = scene_config["table"]
    level_count = len(cylinder_level_counts)
    source_center = [
        pallet["pose"][0],
        pallet["pose"][1],
        pallet["size"][2]
        + cylinder["diameter"] / 2
        + cylinder["vertical_pitch"] * (level_count - 1),
    ]
    table_target = table["target_region"]
    source_support_names = (
        [
            f"stoneware_l{level_count - 1}_{index}"
            for index in range(1, cylinder_level_counts[-2] + 1)
        ]
        if level_count > 1
        else []
    )
    pick_place_demo = Node(
        package="stage1_stoneware_sim",
        executable="stage1_pick_place.py",
        name="stage1_pick_place_demo",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "source_name": f"stoneware_l{level_count}_1",
                "source_support_names": source_support_names,
                "source_center": source_center,
                "table_name": table["model"],
                "table_center": [
                    table_target["center"][0],
                    table_target["center"][1],
                    table_target["cylinder_center_z"],
                ],
                "cylinder_diameter": cylinder["diameter"],
                "use_sim_time": True,
            }
        ],
    )
    suction_bridge_arguments = []
    for pad in range(1, 6):
        contact_topic = (
            f"/world/{scene_config['world']}/model/"
            f"{scene_config['robot']['model']}/link/stage1_suction_manifold/"
            f"sensor/stage1_suction_pad_{pad}_contact/contact"
        )
        suction_bridge_arguments.append(
            f"{contact_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        )
    for cylinder_name in cylinder_names:
        suction_bridge_arguments.extend(
            [
                f"/stage1/suction/attach/{cylinder_name}"
                "@std_msgs/msg/Empty]gz.msgs.Empty",
                f"/stage1/suction/detach/{cylinder_name}"
                "@std_msgs/msg/Empty]gz.msgs.Empty",
                f"/model/{cylinder_name}/detachable_joint/state"
                "@std_msgs/msg/String[gz.msgs.StringMsg",
            ]
        )
    suction_bridge_arguments.append(
        "/world/stage1_raw_stoneware/material_color"
        "@ros_gz_interfaces/msg/MaterialColor]gz.msgs.MaterialColor"
    )
    suction_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=suction_bridge_arguments,
        output="screen",
    )
    suction_controller = Node(
        package="stage1_stoneware_sim",
        executable="stage1_suction_controller.py",
        parameters=[
            {
                "cylinder_names": cylinder_names,
                "world_name": scene_config["world"],
                "robot_model_name": scene_config["robot"]["model"],
            }
        ],
        output="screen",
    )

    def start_description_republisher(event, context):
        if event.returncode == 0:
            return [robot_description_republisher, suction_controller]
        return [Shutdown(reason="Stage 1 robot spawn failed.")]

    def start_robot_nodes(event, context):
        if event.returncode == 0:
            return [robot_state_publisher, joint_state_broadcaster_spawner]
        return [Shutdown(reason="Gazebo controller manager did not initialize.")]

    def start_moveit(event, context):
        if event.returncode == 0:
            if (
                LaunchConfiguration("run_first_pick_place")
                .perform(context)
                .lower()
                == "true"
            ):
                return [trajectory_controller_spawner]
            return [move_group, rviz, planning_scene_publisher]
        return [Shutdown(reason="Stage 1 joint-state broadcaster startup failed.")]

    def start_moveit_with_execution(event, context):
        if event.returncode == 0:
            return [move_group, rviz, planning_scene_publisher]
        return [
            Shutdown(reason="Stage 1 trajectory controller startup failed.")
        ]

    def start_pick_place(event, context):
        if (
            LaunchConfiguration("run_first_pick_place")
            .perform(context)
            .lower()
            != "true"
        ):
            return []
        if event.returncode == 0:
            return [pick_place_demo]
        return [Shutdown(reason="Stage 1 planning scene startup failed.")]

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string",
            robot_description,
            "-name",
            "ur",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo with its graphical interface.",
            ),
            DeclareLaunchArgument(
                "rviz_enabled",
                default_value="true",
                description="Start RViz with the MoveIt planning panel.",
            ),
            DeclareLaunchArgument(
                "run_first_pick_place",
                default_value="false",
                description=(
                    "Enable trajectory execution and run one top-cylinder "
                    "pick-and-place motion."
                ),
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
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
            suction_bridge,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=start_description_republisher,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=robot_description_republisher,
                    on_exit=start_robot_nodes,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=start_moveit,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=trajectory_controller_spawner,
                    on_exit=start_moveit_with_execution,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=planning_scene_publisher,
                    on_exit=start_pick_place,
                )
            ),
            LogInfo(
                msg=IfElseSubstitution(
                    LaunchConfiguration("run_first_pick_place"),
                    if_value=(
                        "Stage 1: first pick-and-place motion enabled; "
                        "scaled trajectory controller will execute the arm path."
                    ),
                    else_value=(
                        "Stage 1: MoveIt planning enabled; trajectory execution "
                        "and the arm trajectory controller are disabled."
                    ),
                )
            ),
        ]
    )
