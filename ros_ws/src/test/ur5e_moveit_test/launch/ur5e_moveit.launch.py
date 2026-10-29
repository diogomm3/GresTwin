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
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _build_robot_description(ur_simulation_share):
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
    ur_description_share = get_package_share_directory("ur_description")
    package_share = get_package_share_directory("ur5e_moveit_test")
    ur_simulation_share = get_package_share_directory("ur_simulation_gz")
    urdf_file = os.path.join(ur_description_share, "urdf", "ur.urdf.xacro")
    world_file = os.path.join(package_share, "worlds", "ur5e.world.sdf")

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

    robot_description_content = _build_robot_description(ur_simulation_share)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description_content},
        ],
    )
    robot_description_republisher = Node(
        package="ur5e_moveit_test",
        executable="republish_robot_description.py",
        name="robot_description_republisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
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

    def start_description_republisher(event, context):
        if event.returncode == 0:
            return [robot_description_republisher]
        return [
            LogInfo(msg="Robot spawning failed; description relay will not start."),
            Shutdown(reason="Robot spawning failed."),
        ]

    def start_robot_nodes(event, context):
        if event.returncode == 0:
            return [robot_state_publisher, joint_state_broadcaster_spawner]
        return [
            LogInfo(
                msg="Controller-manager initialization failed; "
                "ROS robot nodes will not start."
            ),
            Shutdown(reason="Controller-manager initialization failed."),
        ]

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
            gazebo,
            spawn_robot,
            clock_bridge,
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
                        Shutdown(
                            reason="UR5e pick-and-place demonstration finished."
                        )
                    ],
                )
            ),
        ]
    )
