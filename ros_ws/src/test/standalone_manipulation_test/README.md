# Standalone robot manipulation test

This `ament_cmake` package is a standalone robot manipulation test under `ros_ws/src/test/`. It is intentionally separate from future production-line packages. The current model is a positioning-arm foundation; object handling is added in the subsequent demo milestone.

## Robot architecture

The arm uses three independently actuated axes:

- `yaw_joint`: revolute about world Z, limited to `[-pi, pi]` radians.
- `radial_joint`: prismatic along the rotating X axis, with 0.30 m of fixed reach plus 0.00–0.70 m of slide.
- `lift_joint`: prismatic along Z, limited to `[-0.35, 0.25]` m.

The fixed base and rail place the tool frame at `z = 0.68 m + lift_joint`; the tool center can therefore reach approximately 0.33–0.93 m above the ground at radii of 0.30–1.00 m and any yaw angle. This is the kinematic workspace before accounting for link/table collisions. The downward-facing suction-pad-shaped tool is fixed to the lift carriage. Workpiece handling is not implemented yet; the planned grasp/release uses Gazebo's detachable fixed-joint system, not a force-based suction simulation.

## RViz visualization

## RViz visualization

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 launch stoneware_manipulation_test view_robot.launch.py
```

The launch also runs `joint_state_publisher` to provide default joint positions so all links are visible. This is a visualization setup only; it does not command or simulate the robot.

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

The Gazebo simulation loads a 1 kHz controller manager alongside 1 kHz world physics. The arm is controlled with a `JointTrajectoryController` action using joint order `yaw_joint`, `radial_joint`, `lift_joint`. The manipulation sequence will use named Cartesian poses and calculate these joint targets analytically for this cylindrical geometry; it will not depend on manually entered joint commands.

The current simulation is a robot/controller smoke test; the standalone pick-and-place sequence and workpiece are added in the next milestone.

Process the model from the repository root with:

```bash
xacro ros_ws/src/test/standalone_manipulation_test/urdf/stoneware_arm.urdf.xacro
```
