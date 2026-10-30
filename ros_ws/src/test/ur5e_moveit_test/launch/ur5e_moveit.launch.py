import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _add_stage1_end_effector(robot):
    if not any(link.get("name") == "tool0" for link in robot.findall("link")):
        raise RuntimeError("The UR5e description does not contain the expected tool0 link")

    link = ET.SubElement(robot, "link", {"name": "stage1_suction_manifold"})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "2.0"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "0.0007",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.118",
            "iyz": "0",
            "izz": "0.118",
        },
    )

    beam = ET.SubElement(link, "visual", {"name": "stage1_manifold_beam"})
    ET.SubElement(beam, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    beam_geometry = ET.SubElement(beam, "geometry")
    ET.SubElement(beam_geometry, "box", {"size": "0.84 0.065 0.035"})
    beam_material = ET.SubElement(beam, "material", {"name": "stage1_manifold_gray"})
    ET.SubElement(beam_material, "color", {"rgba": "0.22 0.25 0.27 1"})
    beam_collision = ET.SubElement(link, "collision", {"name": "stage1_beam_collision"})
    ET.SubElement(beam_collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    beam_collision_geometry = ET.SubElement(beam_collision, "geometry")
    ET.SubElement(beam_collision_geometry, "box", {"size": "0.84 0.065 0.035"})

    for index, position in enumerate((-0.36, -0.18, 0.0, 0.18, 0.36), start=1):
        for element_name, name in (
            ("visual", f"stage1_suction_pad_{index}"),
            ("collision", f"stage1_suction_pad_{index}_collision"),
        ):
            pad = ET.SubElement(link, element_name, {"name": name})
            ET.SubElement(
                pad,
                "origin",
                {"xyz": f"{position} 0 -0.0275", "rpy": "0 0 0"},
            )
            geometry = ET.SubElement(pad, "geometry")
            ET.SubElement(geometry, "cylinder", {"radius": "0.035", "length": "0.02"})
            if element_name == "visual":
                material = ET.SubElement(
                    pad, "material", {"name": "stage1_suction_pad_rubber"}
                )
                ET.SubElement(material, "color", {"rgba": "0.08 0.09 0.10 1"})

    joint = ET.SubElement(
        robot,
        "joint",
        {"name": "stage1_suction_manifold_fixed_joint", "type": "fixed"},
    )
    ET.SubElement(joint, "parent", {"link": "tool0"})
    ET.SubElement(joint, "child", {"link": "stage1_suction_manifold"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0.04", "rpy": "0 0 0"})


def _build_robot_description(
    ur_simulation_share,
    stage1_end_effector=False,
    enable_detachable_joint=True,
):
    xacro = shutil.which("xacro")
    if xacro is None:
        raise RuntimeError("The xacro executable is not available in PATH")

    controllers_file = os.path.join(
        ur_simulation_share, "config", "ur_controllers.yaml"
    )
    description_file = os.path.join(
        ur_simulation_share, "urdf", "ur_gz.urdf.xacro"
    )
    result = subprocess.run(
        [
            xacro,
            description_file,
            "safety_limits:=true",
            "safety_pos_margin:=0.15",
            "safety_k_position:=20",
            "name:=ur",
            "ur_type:=ur5e",
            f"simulation_controllers:={controllers_file}",
        ],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    root = ET.fromstring(result.stdout)
    gazebo = next(
        (
            element
            for element in root.findall("gazebo")
            if "reference" not in element.attrib
        ),
        None,
    )
    if gazebo is None:
        gazebo = ET.SubElement(root, "gazebo")

    if stage1_end_effector:
        _add_stage1_end_effector(root)

    if enable_detachable_joint:
        plugin = ET.SubElement(
            gazebo,
            "plugin",
            {
                "filename": "gz-sim-detachable-joint-system",
                "name": "gz::sim::systems::DetachableJoint",
            },
        )
        for tag, value in (
            ("parent_link", "wrist_3_link"),
            ("child_model", "workpiece"),
            ("child_link", "cylinder_link"),
            ("detach_topic", "/workpiece/detach"),
            ("attach_topic", "/workpiece/attach"),
            ("output_topic", "/workpiece/state"),
        ):
            ET.SubElement(plugin, tag).text = value
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    package_share = get_package_share_directory("ur5e_moveit_test")
    world_file = os.path.join(package_share, "worlds", "ur5e.world.sdf")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="false",
                description="Start Gazebo with its graphical interface.",
            ),
            DeclareLaunchArgument(
                "rviz_enabled",
                default_value="false",
                description="Start RViz with the MoveIt configuration.",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=world_file,
                description="Gazebo world file used for the UR5e simulation.",
            ),
            DeclareLaunchArgument(
                "run_pick_place",
                default_value="false",
                description="Run the collision-aware workpiece pick-and-place test.",
            ),
            DeclareLaunchArgument(
                "moveit_enabled",
                default_value="true",
                description="Start MoveGroup after the UR5e controllers are active.",
            ),
            DeclareLaunchArgument(
                "stage1_end_effector",
                default_value="false",
                description="Add the fixed five-pad Stage 1 fixture to tool0.",
            ),
            DeclareLaunchArgument(
                "enable_detachable_joint",
                default_value="true",
                description="Enable the existing simulation-only workpiece attachment.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context):
    ur_description_share = get_package_share_directory("ur_description")
    ur_simulation_share = get_package_share_directory("ur_simulation_gz")
    urdf_file = os.path.join(ur_description_share, "urdf", "ur.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=urdf_file,
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

    robot_description_content = _build_robot_description(
        ur_simulation_share,
        stage1_end_effector=(
            LaunchConfiguration("stage1_end_effector").perform(context).lower()
            == "true"
        ),
        enable_detachable_joint=(
            LaunchConfiguration("enable_detachable_joint").perform(context).lower()
            == "true"
        ),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description_content},
        ],
    )
    world = LaunchConfiguration("world_file")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                LaunchConfiguration("gazebo_gui"),
                if_value=[" -r -v 4 ", world],
                else_value=[" -s -r -v 4 ", world],
            )
        }.items(),
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/workpiece/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/workpiece/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/workpiece/state@std_msgs/msg/String[gz.msgs.StringMsg",
            "/model/workpiece/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
        ],
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
        ],
        output="screen",
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": True,
                "publish_robot_description_semantic": True,
                "trajectory_execution": {
                    "allowed_start_tolerance": 0.03,
                },
            },
        ],
        condition=IfCondition(LaunchConfiguration("moveit_enabled")),
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("ur_moveit_config"), "config", "moveit.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("rviz_enabled")),
    )
    pick_place_demo = Node(
        package="ur5e_moveit_test",
        executable="pick_place_demo.py",
        name="ur5e_pick_place_demo",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("run_pick_place")),
    )

    def start_robot_nodes(event, context):
        if event.returncode == 0:
            return [joint_state_broadcaster_spawner]
        return [
            LogInfo(
                msg="Robot spawning failed; controller spawners will not start."
            ),
            Shutdown(reason="Robot spawning failed."),
        ]

    return [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        clock_bridge,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=start_robot_nodes,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[trajectory_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=trajectory_controller_spawner,
                on_exit=[move_group, rviz, pick_place_demo],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=pick_place_demo,
                on_exit=[
                    Shutdown(reason="UR5e pick-and-place demonstration finished.")
                ],
            )
        ),
    ]
