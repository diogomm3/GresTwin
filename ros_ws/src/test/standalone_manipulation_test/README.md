# Standalone Robot Manipulation Test

This package is an isolated ROS 2 Jazzy and Gazebo Harmonic example for one cylindrical pick-and-place cycle. It lives under `ros_ws/src/test/` so it remains separate from future production-line packages. It does not implement a conveyor, machine stations, stacks, vision, or force-based grasping.

## Robot and Workspace

The robot is a fixed-base cylindrical positioning arm with three independently actuated axes:

| Joint | Axis | Limits | Motion |
| --- | --- | --- | --- |
| `yaw_joint` | Revolute about world Z | `[-pi, pi]` rad | Full rotation |
| `radial_joint` | Prismatic along rotating X | `[-0.35, 0.35]` m | 0.65 m nominal radius plus slide |
| `lift_joint` | Prismatic along Z | `[-0.165, 0.435]` m | Vertical tool travel |

The tool-center workspace before collision checks is approximately radius `0.30-1.00 m`, any yaw angle, and height `0.33-0.93 m`. The zero joint pose places the tool at the pickup contact point `(0.65, 0, 0.495) m`; this also gives Gazebo's detachable fixed joint the correct initial relative transform. The model uses a 0.70 m base, a horizontal rail, a radial carriage, a vertical lift, and a fixed suction-pad-shaped tool. This is a positioning demonstration, not an industrial robot model. The geometric workspace does not account for self-collision or arbitrary obstacle avoidance.

The workpiece is a dynamic cylinder, 0.10 m diameter by 0.12 m tall, mass 0.20 kg. Its pickup center is `(0.65, 0, 0.42) m`; its destination center is `(0, 0.65, 0.42) m`. Both tables have a 0.36 m top surface, so the cylinder rests on them at those poses.

The tool is fixed to the lift carriage. Grasp and release use Gazebo Harmonic's `DetachableJoint` system: Gazebo creates/removes a fixed constraint between the lift link and cylinder. The demo first detaches the plugin's initially attached state, verifies the pickup pose, then attaches after approaching the cylinder. This is a simulated constraint, not suction or force-based grasping.

## Motion

MoveIt 2 is available in the Jazzy environment but is not used for this example. The robot's three-axis cylindrical kinematics have a direct inverse mapping from Cartesian position to yaw, radius, and lift; the demo sends velocity-limited joint trajectories to `JointTrajectoryController`. This avoids building an SRDF, planning pipeline, kinematics plugin, and MoveIt controller configuration for a robot that needs no orientation control. It does not perform general collision-aware planning: the named approach, lift, transfer, place, and retreat poses are validated for this fixed empty scene. MoveIt becomes appropriate when the example needs obstacle-aware paths, orientation constraints, or additional joints.

## Dependencies and Build

Install or verify ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `ros2_control`, `joint_state_broadcaster`, and `joint_trajectory_controller`. MoveIt is optional for this demo. The repository's [environment setup](../../../../AGENT.md) lists the ROS packages used by the project.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

Expand and validate the robot description:

```bash
xacro src/test/standalone_manipulation_test/urdf/stoneware_arm.urdf.xacro -o /tmp/stoneware_arm.urdf
check_urdf /tmp/stoneware_arm.urdf
```

## Run the Demo

From `ros_ws` with the overlay sourced, start the GUI simulation and one cycle:

```bash
ros2 launch stoneware_manipulation_test demo.launch.py
```

For a headless run, such as on a CI machine:

```bash
ros2 launch stoneware_manipulation_test demo.launch.py headless:=true
```

The launch starts one world containing the robot's environment and a single cylinder, publishes the robot description, spawns the robot, waits for successful creation, activates both controllers sequentially, then runs the state machine. The run exits explicitly with `DEMO_STATE SUCCEEDED` or `DEMO_STATE FAILED`; a nonzero demo process status indicates failure. A new launch creates a new world, so repeated runs do not accumulate workpieces.

## Automated Validation

Run the headless end-to-end check from a sourced `ros_ws` overlay:

```bash
ros2 run stoneware_manipulation_test validate_demo.py
```

The validator requires a zero launch exit code and checks that every expected state appears in order, including grasp, lift, transfer, release, and safe return. The demo itself also checks controller availability, joint feedback, trajectory action results, attachment state, and measured cylinder poses at pickup, lift, destination, and release.

## Inspect and Troubleshoot

While the simulation is running, inspect active controllers and joint feedback:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /manipulation_demo/state
```

`arm_controller` and `joint_state_broadcaster` should be `active`. The arm action is `/arm_controller/follow_joint_trajectory`. Gazebo's 10 ms physics step and the controller's 100 Hz update rate are intentionally matched; no update-period mismatch warning was observed in the successful runs.

- If controller actions are unavailable, confirm both controllers are `active` and that the `gz_ros2_control` plugin loaded from the robot description.
- If Gazebo is slow to start, close other Gazebo instances before running the validator; the test uses real-time state and controller readiness checks rather than fixed startup sleeps.
- If the grasp state does not become `attached`, inspect the `/workpiece/attach` bridge and the `DetachableJoint` topic configuration in the Xacro.
- If a pose assertion fails, check table, cylinder, tool, and joint-limit dimensions together. The pickup and destination tables are part of the validated scene.
- Gazebo may print a `gz_ros2_control` initialization warning about an executor not being available during hardware initialization; the hardware still configures and activates successfully in the verified run.
