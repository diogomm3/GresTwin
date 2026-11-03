# Stage 1: Raw Stoneware Pickup and Transfer

This package is a standalone Gazebo Harmonic scene for preparing the first
production workstation. It reuses the repository's UR5e launch and adds
stage-local pallet, stoneware-cylinder, and placement-table models.

## Status

Implemented in this milestone:

- A UR5e with a five-pad suction manifold fixed to `tool0`.
- A reusable 0.8 m x 0.5 m x 0.3 m pallet model with two fixed side barriers.
- Fifteen dynamic 0.70 m x 0.10 m stoneware cylinders in a five-level,
  staggered stack.
- A reusable 1.05 m x 0.50 m placement table with a visible 0.95 m x 0.40 m
  target region.
- A static scene validator for dimensions, physical parameters, model count,
  level counts, geometric supports, barrier clearances, and initial overlaps.
- Gazebo contact sensors on each suction pad and stoneware cylinder.

A top-down pick-and-place sequence processes all 15 cylinders by complete
level, starting at `stoneware_l5_1` and continuing down to level 1. Each
cylinder follows the existing approach, suction pickup, straight lift, rotated
transfer, and 20 mm table-release hover. Intended contacts with its adjacent
same-level cylinder and two lower-level supports are allowed during pickup and
lift. After release, the arm retreats and immediately proceeds to the next
cylinder while a five-simulated-second timer removes the placed cylinder from
Gazebo and MoveIt's planning scene. The arm returns to its initial position
once, after the last pickup. All motion segments request 50% velocity and
acceleration scaling.

The suction controller clears cached pad contacts when suction changes state
and ignores contact updates while suction is off, preventing a later pickup
from using stale contact data. The simulated pickup remains an idealized fixed
attachment after pad contact, not a physically validated suction grasp.
Cutting behavior is not implemented. The full 15-cylinder sequence has not yet
been verified in an end-to-end simulation run.

The side barriers are integral to the static pallet model. Their height is
0.40 m from the pallet bottom. Their inner faces align with the pallet's
0.5 m-wide deck edge and are tangent to the outermost bottom-level cylinders;
the outer faces extend 10 mm beyond the deck width.

The current layout places the table at `(-0.15, 0.50, 0)` m, the UR5e base at
`(0.50, 0.50, 0.55)` m, and the pallet at `(1.18, 0.50, 0)` m. The 0.16 m
square pedestal supports the measured 0.151 m UR5e base footprint and is
0.55 m high, level with the tabletop. Clearances are 45 mm from table to
pedestal and 200 mm from pedestal to pallet. The reach figures are geometric
estimates, not proof of collision-free MoveIt paths.

## Model provenance and license

No pallet model was present in the repository or the local user model folders.
A Gazebo Fuel search also did not return a suitable pallet candidate. The
pallet is therefore an original model authored for this package from primitive
boxes; it is not derived from or redistributed from a third-party mesh. The
placement table and stoneware cylinder are likewise original primitive
geometry. These assets are covered by the repository's Apache-2.0 license.

The robot uses the official Universal Robots ROS 2 `ur_description`,
`ur_simulation_gz`, and `ur_moveit_config` packages, through the existing
`ur5e_moveit_test` launch. Stage 1 adds a locally authored five-pad suction
manifold to the robot description without modifying the upstream UR assets.

## Geometry and physical assumptions

`config/stage1_scene.json` holds the scene's configured dimensions and layout.
The included model SDF files define collision geometry and physical
properties; `scripts/validate_stage1.py` checks their collision extents against
the configuration.

| Item | Configuration |
| --- | --- |
| Pallet | 0.80 m long x 0.50 m wide x 0.30 m high |
| Robot pedestal | 0.16 m x 0.16 m x 0.55 m; UR5e base footprint about 0.151 m |
| Side barriers | 0.80 m long x 0.02 m thick x 0.40 m high; fixed to pallet |
| Cylinder | 0.70 m long x 0.10 m diameter |
| Cylinder density assumption | 45.473 kg/m^3 (simulation-only PoC value) |
| Cylinder mass | 0.250 kg |
| Cylinder inertia (axis / transverse) | 0.000313 / 0.010365 kg m^2 |
| Bullet friction coefficient | 0.90 on cylinders; 1.00 on pallet top |
| Cylinder / pallet-top rolling friction coefficient | 0.05 |
| Cylinder / pallet-top restitution | 0.02 |
| Placement table | 1.05 m x 0.50 m; tabletop at z=0.55 m |
| Placement target region | 0.95 m x 0.40 m, centered at (-0.15, 0.50) m |

Density, friction, restitution, and mass are simulation assumptions, not
experimentally measured stoneware properties. The cylinder retains its full,
solid visual and collision geometry; the 0.25 kg mass is a deliberately reduced
digital payload for the pickup proof of concept, not a hollow-cylinder model
or a claim about real stoneware density. The density-based mass and solid
cylinder inertia are calculated from the configured dimensions. The SDF
values are the tuning points. The rolling-resistance value is a simulation
assumption intended to limit free rolling; it is not a measurement of clay or
pallet friction. The Bullet contact model also uses a small restitution value
to reduce bouncing. No deformability, plasticity, or moisture behavior is
simulated.

Each cylinder also exposes a Gazebo contact-sensor topic so contact can be
observed separately from a future grasp decision.

The pallet is at `(1.18, 0.50, 0)` m. Its upper deck is at z=0.30 m. All
cylinders are parallel to its 0.8 m length. Within each level, center spacing
is 0.10 m; each upper level is shifted by half a diameter and raised by
`sqrt(3) * 0.05 = 0.086603` m. This puts each upper cylinder in two-cylinder
support contact without cross-sectional overlap. The base row spans exactly
0.50 m across the pallet and is tangent to the barrier inner faces. The initial
cylinder centers are:

| Level from bottom | Count | Center z (m) | Center y relative to pallet (m) |
| --- | ---: | ---: | --- |
| 1 | 5 | 0.350000 | -0.20 through 0.20, 0.10 m spacing |
| 2 | 4 | 0.436603 | -0.15 through 0.15, 0.10 m spacing |
| 3 | 3 | 0.523205 | -0.10 through 0.10, 0.10 m spacing |
| 4 | 2 | 0.609808 | -0.05, 0.05 |
| 5 | 1 | 0.696410 | 0 |

With the UR5e shoulder about 0.163 m above its base, the top-cylinder center is
about 0.680 m from the shoulder axis and the table target center about 0.660 m
away, within the configured 0.85 m nominal reach. These remain geometric
estimates, not proof that IK and collision-checked approach paths will succeed.

## Suction tool PoC

The Stage 1 robot has a fixed manifold with five collision-enabled pads spaced
along the cylinder length. Each pad has a Gazebo contact sensor. Publish a
Boolean command to enable or disable suction:

```bash
ros2 topic pub --once /stage1/suction/command std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /stage1/suction/command std_msgs/msg/Bool "{data: false}"
```

When enabled, contact with a stoneware cylinder dynamically loads a detachable
joint for that cylinder, attaching it to the manifold. No cylinder joints are
loaded at startup. Disabling suction removes the joint and releases the
cylinder. Pads are red while suction is off and green while it is on. The
`/stage1/suction/enabled` and `/stage1/suction/attached` Boolean topics report
the controller state. Suction may be enabled before making contact; the
controller attaches the first cylinder reported by a pad sensor.

Gazebo Harmonic provides contact sensors / touch events and
`DetachableJoint`, but this PoC does not simulate vacuum pressure, seal quality,
pad compliance, or grip-force limits. It demonstrates contact-triggered
pick/release control only; the reduced simulation mass is not evidence that a
real suction tool can hold a cylinder of the same dimensions.

## Build and launch

Install ROS 2 Jazzy, Gazebo Harmonic, `ros_gz`, `gz_ros2_control`, and the UR
Jazzy packages. The root [AGENT.md](../../../AGENT.md) documents the project's
Ubuntu 24.04 setup. The relevant UR packages are `ur_description`,
`ur_simulation_gz`, and `ur_moveit_config`.

On Ubuntu 24.04, install the UR packages with:

```bash
sudo apt install \
  ros-jazzy-ur-description \
  ros-jazzy-ur-simulation-gz \
  ros-jazzy-ur-moveit-config
```

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-up-to stage1_stoneware_sim
source install/setup.bash
```

Start the GUI scene:

```bash
ros2 launch stage1_stoneware_sim stage1.launch.py
```

For a headless launch:

```bash
ros2 launch stage1_stoneware_sim stage1.launch.py gazebo_gui:=false
```

To build Stage 1 and start its headless launch in one step, run this script
from any directory:

```bash
./ros_ws/src/stage1/run.sh
```

To run the top-down pick-and-place sequence:

```bash
./ros_ws/src/stage1/run.sh run_first_pick_place:=true
```

For a bounded run, set `pick_place_max_cylinders` (for example, `:=2` to pick
L5 and one L4 cylinder).

The single-cylinder diagnostic copy remains available with
`run_first_pick_place_test:=true`.

The script selects and prints an isolated ROS domain and Gazebo partition when
they are not already set. This prevents unrelated or stale ROS/Gazebo
processes from sharing Stage 1's controller-manager services and simulation
topics. If you need to inspect or control the running Stage 1 nodes from
another terminal, use the printed `ROS_DOMAIN_ID` and `GZ_PARTITION` there.
Explicitly set values are preserved.

To view the pallet and complete cylinder stack without spawning the UR5e,
launch the same world directly in Gazebo:

```bash
ros2 launch stage1_stoneware_sim stage1_stack_only.launch.py
```

Add `gazebo_gui:=false` for a headless stack-only run. Stop the robot-enabled
Stage 1 simulation before starting this one so both Gazebo instances do not
publish into the same world topic namespace.

The launch sets `GZ_SIM_RESOURCE_PATH` to the installed reusable model library
and starts MoveGroup and RViz alongside the Stage 1 world. RViz is enabled by
default; pass `rviz_enabled:=false` to disable it. The robot remains at its
configured base pose with an unmodified `tool0` end. MoveIt trajectory execution is
disabled, so plans can be visualized but cannot command arm movement. The launch
copies the world collision geometry into MoveIt's planning scene from the
Stage 1 world and model SDF files, so the pallet, stack, table, pedestal, and
target marker are visible to the MoveIt RViz display and included in planning.

## Validate

Run the model/configuration checks:

```bash
ros2 run stage1_stoneware_sim validate_stage1.py
```

The validator checks actual collision extents in the model SDF files, cylinder
length/diameter, calculated mass and inertia, friction and restitution,
15-instance count, all five level counts, pairwise non-overlap, two supports
per upper cylinder, fixed barrier dimensions and clearances, and target/table
alignment.

To inspect resolved model dimensions and simulation state, start the world
through the launch above and use Gazebo's model and topic tools. The expected
initial count is 15. A headless 300-second check of the pallet and cylinders
without a robot can be run with:

```bash
cd ros_ws
GZ_SIM_RESOURCE_PATH="$PWD/src/stage1/models" \
  gz sim -s -r --iterations 150000 src/stage1/worlds/stage1_raw_stoneware.world.sdf
```

The world uses a 0.002 s step, so 150,000 iterations is 300 simulated seconds.
This stack-only run does not satisfy the full-scene test with the UR5e present.
The current full-scene run failed early as reported above; investigate its
cause before treating a later 300-second run as successful. Inspect the pose
stream for all 15 models and report maximum displacement, tipping, pallet
escape, and any fall-through.
Use the existing UR5e validator in a separate terminal to check the robot
description, state, and controllers:

```bash
ros2 run ur5e_moveit_test validate_ur5e.py
```

The world pose stream gives the instantiated model names and measured settled
poses:

```bash
gz topic -e -t /world/stage1_raw_stoneware/pose/info -n 1 --json-output
```

Look for the `stoneware_l1_*` through `stoneware_l5_*` models (15 in total).
Contact sensors are named `stoneware_contact_sensor` within each cylinder;
Gazebo's topic listing (`gz topic -l`) shows their resolved model/link sensor
topics.

The robot validator checks state, controller, MoveIt planning, and a reversible
joint-space execution in the existing UR5e world. It is not a Stage 1
pick-and-place test.

In the Milestone 1 regression run, `validate_ur5e.py` passed. The existing
three-axis `validate_demo.py` did not pass: its spawn process timed out waiting
for `/robot_description`, so its controller and pick-and-place sequence were
not validated. No changes were made to that demo.

## Placement consumption

The target region is centered at `(-0.15, 0.50, 0.551)` m and has a visible
`0.95 m x 0.40 m` footprint. The intended cylinder center height on the table
is 0.60 m. The current top-down motion removes each cylinder from Gazebo and
MoveIt's planning scene five simulated seconds after confirmed release while
the robot continues with the next pickup.
