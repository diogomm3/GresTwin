from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("stoneware_manipulation_test")
    world_file = PathJoinSubstitution([package_share, "worlds", "stoneware.world.sdf"])
    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "stoneware_arm.urdf.xacro"]
    )
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "gz_args": [world_file, " -r"],
            "on_exit_shutdown": "true",
        }.items(),
    )
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
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    def start_controllers_after_spawn(event, context):
        if event.returncode == 0:
            return [joint_state_broadcaster_spawner, arm_controller_spawner]
        return [
            LogInfo(msg="Robot spawning failed; controllers will not be started."),
            Shutdown(reason="Robot spawning failed."),
        ]

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            clock_bridge,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=start_controllers_after_spawn,
                )
            ),
        ]
    )
