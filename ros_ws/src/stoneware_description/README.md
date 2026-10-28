# stoneware_description

This `ament_cmake` package contains a simple, simulator-independent Xacro model for the first robot-arm milestone.

The arm has a cylindrical fixed base and two links connected by shoulder and elbow revolute joints. Both arm links use matching visual and collision geometry, with inertial properties for simulation use. The joints rotate about the Y axis, allowing planar motion in the X-Z plane. No Gazebo-specific plugins or pick-and-place behavior are included.

Launch the robot state publisher and RViz with the package's saved display configuration:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 launch stoneware_description view_robot.launch.py
```

The launch also runs `joint_state_publisher` to provide the revolute joints' default positions so all links are visible. This is a visualization setup only; it does not command or simulate the robot.

Process the model from the repository root with:

```bash
xacro ros_ws/src/stoneware_description/urdf/stoneware_arm.urdf.xacro
```
