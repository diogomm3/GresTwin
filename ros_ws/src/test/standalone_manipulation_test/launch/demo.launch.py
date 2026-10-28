from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("stoneware_manipulation_test")
    gazebo_launch = PathJoinSubstitution([package_share, "launch", "gazebo.launch.py"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
    )
    demo = Node(
        package="stoneware_manipulation_test",
        executable="pick_place_demo.py",
        name="pick_place_demo",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            gazebo,
            demo,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=demo,
                    on_exit=[Shutdown(reason="Pick-and-place demonstration finished.")],
                )
            ),
        ]
    )
