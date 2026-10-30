# Stage 1: Raw Stoneware Pickup and Transfer

This package is a standalone Gazebo Harmonic scene for preparing the first
production workstation. Stage 1 owns its launch files, robot-description
helper, pallet, stoneware-cylinder, and placement-table models. It uses the
upstream ROS 2 UR description and Gazebo packages, but does not depend on the
repository's UR5e test package.

## Status

Implemented in this milestone:

- A stationary UR5e in the Stage 1 world, with a five-pad fixture fixed to
  `tool0`.
- A reusable 1.0 m x 0.85 m x 0.25 m pallet model with two fixed side
  barriers.
- Thirty-six dynamic 0.90 m x 0.10 m stoneware cylinders in an eight-level,
  staggered stack.
- A reusable 1.05 m x 0.50 m placement table with a visible 0.95 m x 0.40 m
  target region.
- A static scene validator for dimensions, physical parameters, model count,
  level counts, geometric supports, barrier clearances, and initial overlaps.
- A Gazebo contact sensor on each stoneware cylinder for later contact-aware
  handling.

Not implemented: robot motion, grasp execution, the top-down production
sequence, object removal after placement, cutting behavior, or a validated
physical suction grasp. The intended order is top-down by complete level:
`stoneware_l8_1`, then both `stoneware_l7_*`, then all
`stoneware_l6_*`, continuing down through the bottom level. No sequence runs
yet.

The side barriers are integral to the static pallet model. Their height is
0.40 m from the pallet bottom; the inner faces have 5 mm clearance from the
outermost bottom-level cylinders. The original Stage 1 pallet center was
`(0.62, -0.15, 0)` m. With the wider pallet, that exact center intersects the
unchanged table's near leg, so the current center is `(0.62, -0.18, 0)` m:
the pallet remains at the original X/Z coordinates and is shifted 30 mm in
negative Y, leaving 5 mm between the pallet and table collision geometry.
The UR5e and table poses remain unchanged.

The Stage 1 robot is spawned with its six arm joints mechanically fixed. The
scene does not start controllers or command robot motion; this keeps the robot
from disturbing the dynamic cylinder stack while manipulation is not
implemented.

## Model provenance and license

No pallet model was present in the repository or the local user model folders.
A Gazebo Fuel search also did not return a suitable pallet candidate. The
pallet is therefore an original model authored for this package from primitive
boxes; it is not derived from or redistributed from a third-party mesh. The
placement table and stoneware cylinder are likewise original primitive
geometry. These assets are covered by the repository's Apache-2.0 license.

The robot uses the official Universal Robots ROS 2 `ur_description` and
`ur_simulation_gz` packages. Stage 1's local launch helper builds its robot
description, fixes its arm joints, and adds a locally authored URDF beam and
five pad shapes; it does not replace or copy the UR robot assets.

## Geometry and physical assumptions

`config/stage1_scene.json` holds the scene's configured dimensions and layout.
The included model SDF files define collision geometry and physical
properties; `scripts/validate_stage1.py` checks their collision extents against
the configuration.

| Item | Configuration |
| --- | --- |
| Pallet | 1.00 m long x 0.85 m wide x 0.25 m high |
| Side barriers | 1.00 m long x 0.02 m thick x 0.40 m high; fixed to pallet |
| Cylinder | 0.90 m long x 0.10 m diameter |
| Cylinder density assumption | 2,000 kg/m^3 |
| Cylinder mass | 14.137 kg |
| Cylinder inertia (axis / transverse) | 0.017671 / 0.963094 kg m^2 |
| Bullet friction coefficient | 0.90 on cylinders; 1.00 on pallet top |
| Cylinder / pallet-top rolling friction coefficient | 0.05 |
| Cylinder / pallet-top restitution | 0.02 |
| Placement table | 1.05 m x 0.50 m; tabletop at z=0.55 m |
| Placement target region | 0.95 m x 0.40 m, centered at (-0.15, 0.50) m |

Density, friction, restitution, and mass are simulation assumptions, not
experimentally measured stoneware properties. The density-based mass and solid
cylinder inertia are calculated from the configured dimensions. The SDF
values are the tuning points. The rolling-resistance value is a simulation
assumption intended to limit free rolling; it is not a measurement of clay or
pallet friction. The Bullet contact model also uses a small restitution value
to reduce bouncing. No deformability, plasticity, or moisture behavior is
simulated.

Each cylinder also exposes a Gazebo contact-sensor topic so contact can be
observed separately from a future grasp decision.

The pallet is at `(0.62, -0.18, 0)` m. Its upper deck is at z=0.25 m. All
cylinders are parallel to its 1.0 m length. Within each level, center spacing
is 0.10 m; each upper level is shifted by half a diameter and raised by
`sqrt(3) * 0.05 = 0.086603` m. This puts each upper cylinder in two-cylinder
support contact without cross-sectional overlap. The base row spans 0.80 m
across the 0.85 m pallet, leaving 25 mm to each pallet edge and 5 mm to the
barrier inner faces. The initial cylinder centers are:

| Level from bottom | Count | Center z (m) | Center y relative to pallet (m) |
| --- | ---: | ---: | --- |
| 1 | 8 | 0.300000 | -0.35 through 0.35, 0.10 m spacing |
| 2 | 7 | 0.386603 | -0.30 through 0.30, 0.10 m spacing |
| 3 | 6 | 0.473205 | -0.25 through 0.25, 0.10 m spacing |
| 4 | 5 | 0.559808 | -0.20 through 0.20, 0.10 m spacing |
| 5 | 4 | 0.646410 | -0.15 through 0.15, 0.10 m spacing |
| 6 | 3 | 0.733013 | -0.10 through 0.10, 0.10 m spacing |
| 7 | 2 | 0.819615 | -0.05, 0.05 |
| 8 | 1 | 0.906218 | 0 |

The scene places the UR5e base at the world origin. Using the UR5e shoulder
height of approximately 0.163 m, the top-cylinder center is about 0.984 m
from the shoulder axis; an estimated side-contact pose is about 0.995 m away.
These exceed the UR5e's configured nominal 0.85 m reach. The target-region
ends usable by a 0.90 m cylinder are within about 0.82 m. These are geometric
estimates, not a MoveIt plan. Top-level pickup therefore needs a reach/layout
review before implementing robot motion. The Stage 1 launch disables MoveGroup
because this milestone does not plan or command motion.

## End effector and grasp limitations

Stage 1 adds a fixed 0.84 m manifold beam and five 0.07 m diameter pad shapes
at 0.18 m spacing. The link has collision geometry and an explicit inertial
estimate. The fixture is
visual/mechanical design only: there is no vacuum source, compliant lip,
pressure model, seal detection, contact-conditioned attachment, or grasp
controller. Its `tool0` mounting transform and required wrist orientation
still need calibration against the actual URDF and workpiece.

Gazebo Harmonic provides contact sensors / touch events and
`DetachableJoint`, but no stock vacuum-gripper or automatic contact-grasp
system. `Contact` publishes contacts only for configured contact sensors; it
does not create a grasp. `DetachableJoint` creates a rigid fixed constraint
and is not a suction model. Stage 1 does not load that plugin, to avoid
implying that cylinders are being grasped.
The original UR5e demo's fixed attachment remains its simulation-only pickup
mechanism.

For Milestone 2, the available choices are (a) a custom contact/seal and
pressure-force plugin, which is more representative but requires validation
and tuning; (b) an external controller that requests a fixed attachment only
after sensor-confirmed contact, which is robust but still an idealized rigid
grasp; or (c) a third-party suction plugin, after checking compatibility and
license. None is claimed or implemented here as physically validated.

## Build and launch

Install ROS 2 Jazzy, Gazebo Harmonic, `ros_gz`, and the official UR Jazzy
description/simulation packages. The root [AGENT.md](../../../AGENT.md)
documents the project's Ubuntu 24.04 setup. Stage 1 does not depend on
MoveIt or the repository's UR5e test package.

On Ubuntu 24.04, install the UR packages with:

```bash
sudo apt install \
  ros-jazzy-ur-description \
  ros-jazzy-ur-simulation-gz
```

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros_ws
rosdep install --from-paths src/stage1 --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select stage1_stoneware_sim
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

To view the pallet and complete cylinder stack without spawning the UR5e,
launch the same world directly in Gazebo:

```bash
ros2 launch stage1_stoneware_sim stage1_stack_only.launch.py
```

Add `gazebo_gui:=false` for a headless stack-only run. Stop the robot-enabled
Stage 1 simulation before starting this one so both Gazebo instances do not
publish into the same world topic namespace.

The launch sets `GZ_SIM_RESOURCE_PATH` to the installed reusable model library,
starts Gazebo with the Stage 1 world, and spawns the locally configured
stationary UR5e. The six arm joints are fixed and there are no controllers, so
the robot cannot move into the dynamic cylinder stack. The stack-only and
static-inspection launch files are also implemented within Stage 1; the
static-inspection launch shares the local robot-description helper, while the
stack-only launch starts no robot.

## Validate

Run the model/configuration checks:

```bash
ros2 run stage1_stoneware_sim validate_stage1.py
```

The validator checks actual collision extents in the model SDF files, cylinder
length/diameter, calculated mass and inertia, friction and restitution,
36-instance count, all eight level counts, pairwise non-overlap, two supports
per upper cylinder, fixed barrier dimensions and clearances, and target/table
alignment.

To inspect resolved model dimensions and simulation state, start the world
through the launch above and use Gazebo's model and topic tools. The expected
initial count is 36. A headless 300-second check of the pallet and cylinders
without a robot can be run with:

```bash
cd ros_ws
GZ_SIM_RESOURCE_PATH="$PWD/src/stage1/models" \
  gz sim -s -r --iterations 150000 src/stage1/worlds/stage1_raw_stoneware.world.sdf
```

The world uses a 0.002 s step, so 150,000 iterations is 300 simulated seconds.
To test the full Stage 1 launch headlessly, start it with
`gazebo_gui:=false` instead, then inspect the pose stream for all 36 cylinders
throughout the run. Compare positions against the baseline after initial
settling and check for pallet escape, falls, excessive drift, and tipping.
In a 49.1-second headless run of the full launch, all 36 cylinders remained on
the pallet, above its deck, and supported; maximum center displacement from
the configured positions was 5.5 mm. No cylinder tilted more than 5 degrees.
This short result is not a substitute for the 300-second check.

The world pose stream gives the instantiated model names and measured settled
poses:

```bash
gz topic -e -t /world/stage1_raw_stoneware/pose/info -n 1 --json-output
```

Look for the `stoneware_l1_*` through `stoneware_l8_*` models (36 in total).
Contact sensors are named `stoneware_contact_sensor` within each cylinder;
Gazebo's topic listing (`gz topic -l`) shows their resolved model/link sensor
topics.

## Future placement consumption

The target region is centered at `(-0.15, 0.50, 0.551)` m and has a visible
`0.95 m x 0.40 m` footprint. The intended cylinder center height on the table
is 0.60 m. Milestone 3 can use this region directly and remove an object about
10 seconds after confirmed placement. There is no removal timer in this
package.
