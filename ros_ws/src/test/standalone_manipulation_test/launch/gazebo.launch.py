from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("stoneware_manipulation_test")
    world_file = PathJoinSubstitution([package_share, "worlds", "stoneware.world.sdf"])
    xacro_file = PathJoinSubstitution([package_share, "urdf", "stoneware_arm.urdf.xacro"])
    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r"],
        condition=UnlessCondition(LaunchConfiguration("headless")),
        name="gazebo",
        output="screen",
        on_exit=Shutdown(reason="Gazebo exited."),
    )
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r", "-s"],
        condition=IfCondition(LaunchConfiguration("headless")),
        name="gazebo",
        output="screen",
        on_exit=Shutdown(reason="Gazebo exited."),
    )
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_stoneware_arm",
        output="screen",
        parameters=[
            {
                "world": "default",
                "topic": "/robot_description",
                "name": "stoneware_manipulator",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
            }
        ],
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    simulation_bridge = Node(
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

    def start_controllers_after_spawn(event, context):
        if event.returncode == 0:
            return [controller_spawner]
        return [
            LogInfo(msg="Robot spawning failed; controllers will not be started."),
            Shutdown(reason="Robot spawning failed."),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run Gazebo without its GUI.",
            ),
            gazebo_gui,
            gazebo_server,
            robot_state_publisher,
            simulation_bridge,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=start_controllers_after_spawn,
                )
            ),
        ]
    )
