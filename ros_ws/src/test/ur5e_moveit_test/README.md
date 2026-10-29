# UR5e Gazebo, MoveIt, and Pick-and-Place Test

This package runs the official UR5e description, Gazebo Harmonic simulation,
ROS 2 control stack, and MoveIt configuration. It is separate from
`standalone_manipulation_test`, which remains the working three-axis fallback.
It validates robot state, controller startup, MoveIt planning and execution,
and an optional workpiece pick-and-place sequence.

The required ROS packages include `ur_description`, `ur_simulation_gz`, and
`ur_moveit_config`, as well as Gazebo Harmonic, MoveIt 2, `ros2_control`, and
`ros_gz`. Install the official Universal Robots ROS 2 simulation and MoveIt
packages for Jazzy before building this package. If those packages are in a
separate workspace overlay, source that overlay after ROS Jazzy and before
building or sourcing this workspace. Binary packages installed in
`/opt/ros/jazzy` need no additional overlay.

The launch uses the official UR description, simulation controller
configuration, and MoveIt configuration. It orchestrates robot spawning and
controller startup sequentially, and supplies the URDF to MoveGroup as a
parameter so startup does not depend on receiving a one-time
`/robot_description` topic message after the state publisher has emitted it.
After spawning, a transient-local description relay republishes until the
Gazebo controller manager is initialized; the state publisher and controller
spawners then start in sequence.
The workpiece is grasped with Gazebo's detachable fixed-joint system. This is a
simulation-only constraint: it is not a physical gripper, nor a force-based or
contact-sensed grasp.

## Build

Install ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, `ros2_control`, `ros_gz`, and
the official Universal Robots Jazzy packages. In a terminal with any required
UR package overlay sourced, build from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
# If built from source in another workspace, source that workspace here:
# source /path/to/ur_overlay/install/setup.bash
cd ros_ws
colcon build --symlink-install --packages-up-to ur5e_moveit_test
source install/setup.bash
```

For every new terminal, source ROS, any required external UR package overlay,
and this workspace's install setup in that order before running the commands
below.

## Headless Simulation

With the workspace overlays sourced, run:

```bash
ros2 launch ur5e_moveit_test ur5e_moveit.launch.py
```

Gazebo's GUI and RViz are disabled by default. To set the options explicitly:

```bash
ros2 launch ur5e_moveit_test ur5e_moveit.launch.py gazebo_gui:=false rviz_enabled:=false
```

## Graphical Simulation

```bash
ros2 launch ur5e_moveit_test ur5e_moveit.launch.py gazebo_gui:=true rviz_enabled:=true
```

The launch arguments are:

| Argument | Default | Description |
| --- | --- | --- |
| `gazebo_gui` | `false` | Start Gazebo with its graphical interface. |
| `rviz_enabled` | `false` | Start RViz with the MoveIt configuration. |
| `world_file` | The package's `worlds/ur5e.world.sdf` | Use a different Gazebo world file. |
| `run_pick_place` | `false` | Run the automated pick-and-place sequence and shut down the launch afterward. |

The launch starts the UR5e with
`scaled_joint_trajectory_controller` and `joint_state_broadcaster`. Both
must be active before running the validator or demo. Wait for the launch log
to report that `scaled_joint_trajectory_controller` is configured and active,
and for MoveGroup to report `You can start planning now!`. With RViz enabled,
its MotionPlanning panel also reports `Ready to take commands for planning
group ur_manipulator.` The MoveIt planning group is
`ur_manipulator`; its controller mapping targets
`/scaled_joint_trajectory_controller/follow_joint_trajectory`.
The included world selects Bullet Featherstone for mesh collision support and
uses a 2 ms physics step to match the controller's 500 Hz update rate.
The world contains a cylindrical workpiece and pickup/place tables. Pass
`run_pick_place:=true` to run a complete collision-aware MoveIt sequence:

```bash
ros2 launch ur5e_moveit_test ur5e_moveit.launch.py \
  gazebo_gui:=false rviz_enabled:=false run_pick_place:=true
```

The sequence adds the tables and workpiece to the MoveIt planning scene, plans
the approach, lift, transfer, placement, retreat, and safe return, and verifies
the attachment state and workpiece pose at each stage. The launch shuts down
after the demo exits. Confirm success from `DEMO_STATE SUCCEEDED` in the
launch output; a zero launch exit status alone is not sufficient.

To run the demo while inspecting the graphical simulation, leave the
`gazebo_gui:=true rviz_enabled:=true` launch running. In a second terminal,
source the same overlays and run:

```bash
ros2 run ur5e_moveit_test pick_place_demo.py
```

The demo runs against the already launched simulation. Do not start a second
launch for this command.

## Validate Planning and Motion

With the simulation running and both controllers active, open a second
terminal, source the same overlays, and run:

```bash
ros2 run ur5e_moveit_test validate_ur5e.py
```

The validator checks the six-joint robot description and live state, controller
activation and interfaces, then plans and executes a small reversible MoveIt
joint-space motion before returning to the initial state. Each check reports
`PASS`; the final expected result is
`PASS: UR5e controller, state, MoveIt planning, execution, and return validated`.

## Current limitations

- The demo sometimes makes unnecessary arm movements. Path efficiency is
  deferred to a future milestone.
- Pickup alignment is slightly off. Grasp alignment improvements are deferred
  to a future milestone.
- The grasp is implemented with Gazebo's detachable fixed-joint constraint;
  there is no physical gripper, force feedback, or contact-based grasping.
- No 3D sensor is configured for MoveIt's octomap updates. MoveIt may log that
  no 3D sensor plugin is available; this demo instead adds its known tables
  and workpiece directly to the planning scene.

The three-axis demonstration remains available as a fallback. The UR5e
pick-and-place path is separate; the motion-efficiency and pickup-alignment
limitations above are intentionally outside this milestone.
