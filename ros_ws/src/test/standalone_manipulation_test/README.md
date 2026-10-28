# Standalone robot manipulation test

This `ament_cmake` package is a standalone test example under `ros_ws/src/test/`. It contains a simple Xacro robot model, an RViz view, and a Gazebo Harmonic simulation. It is intentionally separate from future production-line packages.

The arm has a cylindrical base fixed to the Gazebo world and two links connected by shoulder and elbow revolute joints. The links have visual, collision, and inertial geometry. Both joints rotate about the Y axis, allowing planar motion in the X-Z plane. `gz_ros2_control` exposes position command and position/velocity state interfaces for the two joints. The simulation includes only a ground plane and directional light; it does not implement object interaction or pick-and-place behavior.

## RViz visualization

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 launch stoneware_manipulation_test view_robot.launch.py
```

The launch also runs `joint_state_publisher` to provide the revolute joints' default positions so all links are visible. This is a visualization setup only; it does not command or simulate the robot.

## Gazebo Harmonic simulation

The package requires the ROS 2 Jazzy binaries for `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `ros2_control`, `joint_state_broadcaster`, and `position_controllers`. See the repository's [environment setup instructions](../../../../README.md) for the ROS apt repository setup. The project is tested with the Gazebo Harmonic version supplied through the Jazzy ROS packages; do not install a separate Gazebo release alongside it.

Build the workspace and launch Gazebo with the robot spawned:

```bash
source /opt/ros/jazzy/setup.bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch stoneware_manipulation_test gazebo.launch.py
```

The launch starts the simulation with a ground plane, publishes the robot description, bridges Gazebo simulation time to ROS, spawns the fixed-base arm, and loads the joint-state broadcaster and position controller only after successful spawning. Gazebo runs with its GUI; stop the launch with **Ctrl+C**.

In another sourced terminal, confirm the controllers are active and inspect the joint states:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

Command both joints in radians by publishing shoulder then elbow positions. This example holds the setpoint for three seconds and moves within the configured joint limits:

```bash
ros2 topic pub --rate 10 --times 30 /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.35, -0.5]}"
```

The controller publishes to `/joint_states`; use `ros2 topic echo /joint_states` to observe the commanded positions. This is a basic position-control demonstration, not a trajectory planner.

Process the model from the repository root with:

```bash
xacro ros_ws/src/test/standalone_manipulation_test/urdf/stoneware_arm.urdf.xacro
```
