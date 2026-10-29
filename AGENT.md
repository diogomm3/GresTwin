# Stoneware Production-Line Simulation

A ROS 2 project to develop each workstation independently, test it, then integrate the workstations into a simulated production line.

## Planned workstations

1. Pick the top cylindrical stoneware blank from a stack and place it at the cutting machine.
2. Detect when a blank has been cut/consumed and advance the remaining material for the next cut.
3. Transfer a cut piece to the next machine, respecting overhead clearance; trigger the machine only after the arm has cleared the area.
4. Pick a plaque, lower until contact/force is detected, release the product, and place it on a conveyor.

These are goals, not implemented features. First, develop and validate a standalone robot manipulation test under `ros_ws/src/test/standalone_manipulation_test/`. Do not begin production-line stations or integration until that focused example is reliable.

## Technology choices

- Windows 11 + **WSL 2 / Ubuntu 24.04 LTS**
- **ROS 2 Jazzy** + **Gazebo Harmonic** + **MoveIt 2 Jazzy**
- **ros2_control** with **gz_ros2_control** for simulated controllers
- RViz for visualization
- Python 3.12 (`rclpy`) for task logic
- URDF/Xacro for the robot description
- Git + GitHub pull requests

Keep task logic behind ROS 2 interfaces and isolate Gazebo-specific worlds, plugins, and physics settings. This makes a future Isaac Sim integration easier, though not automatic.

## Environment setup

> ROS 2 Jazzy's recommended binary packages target Ubuntu 24.04. If your current WSL distro is Debian, install Ubuntu as a separate WSL distro; don't run the Ubuntu installation commands inside Debian.

### 1. Install Ubuntu 24.04

Run in **Windows PowerShell**:

```powershell
wsl --list --online
wsl --install -d Ubuntu-24.04
wsl -l -v
```

Complete Ubuntu's first-run setup, then launch it from the Start menu or run `wsl -d Ubuntu-24.04`.

### 2. Install the compatible ROS stack

Inside the **Ubuntu 24.04** terminal, follow the [official ROS 2 Jazzy Ubuntu binary-package instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), including configuring the ROS apt repository. Ensure Ubuntu's `universe`, `noble-updates`, and `noble-backports` apt suites are enabled, as well as a UTF-8 locale. After configuring the ROS repository, run `sudo apt update` before installing packages. The [ROS/Gazebo compatibility guidance](https://gazebosim.org/docs/harmonic/ros_installation/) recommends Jazzy with Harmonic. Do not add a separate OSRF Gazebo apt repository or mix ROS/Gazebo release versions.

Install the selected ROS packages and development tools:

```bash
sudo apt install \
  ros-jazzy-ros-base \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-dev-tools
```

Source ROS in the current terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

To source ROS automatically in future Bash terminals, add `source /opt/ros/jazzy/setup.bash` to `~/.bashrc` once only.

Verify ROS and run Gazebo's basic smoke test:

```bash
echo $ROS_DISTRO       # expected: jazzy
ros2 --help
gz sim shapes.sdf
```

Gazebo should open its example shapes world. Stop it with **Ctrl+C** in the terminal where it was launched. To check RViz separately, run:

```bash
rviz2
```

WSLg can run Linux GUI applications, but performance depends on graphics drivers and scene complexity. Begin with simple geometry. If either GUI fails, save the error output before trying random driver/package changes.

## Repository setup

Keep the working copy in the Linux filesystem (for example `~/work/stoneware-production-line`), not `/mnt/c/...`, for better build performance.

```bash
mkdir -p ~/work
cd ~/work
git clone <YOUR_GITHUB_REPOSITORY_URL>
cd <YOUR_REPOSITORY_DIRECTORY>
```

Replace the placeholders. Do not put access tokens or passwords in repository URLs.

## Git and pull-request workflow

Do not commit directly to `main`. Use a short-lived branch for each focused change:

```bash
git switch main
git pull --ff-only
git switch -c docs/initial-readme
# Add or update README.md, then review:
git status
git diff --check
git diff
git add README.md
git commit -m "docs: add project setup and roadmap"
git push -u origin docs/initial-readme
```

Open a PR on GitHub from `docs/initial-readme` into `main`. Review the diff and checks, then merge it. After merging:

```bash
git switch main
git pull --ff-only
```

Use clear Conventional Commit-style messages, e.g. `chore: add ROS 2 workspace`, `feat: add initial robot description`, `test: add pick-and-place tests`. Each PR should have one purpose and include how it was verified. Never commit `build/`, `install/`, or `log/` directories.

## Proposed package layout

Add packages incrementally; don't create empty packages for every future feature.

```text
ros_ws/src/
├── test/
│   └── standalone_manipulation_test/  # Self-contained robot manipulation example
├── stoneware_application/    # simulator-independent Python task logic
├── stoneware_interfaces/     # custom ROS interfaces only if needed
└── stoneware_gazebo/         # worlds and Gazebo-specific integration
```

The test directory is a source grouping, not a ROS package; keep package manifests in leaf package directories so colcon can discover them. Add production-line packages and sensors only after the standalone manipulation example is validated. If Isaac Sim is adopted, add a separate simulator integration rather than putting simulator-specific calls in the application layer.

## First milestone

1. Build a standalone three-axis robot manipulation test.
2. Load a small Gazebo world with a single cylindrical workpiece.
3. Demonstrate and verify one complete pick-and-place cycle.
4. Keep production-line stations and multi-object workflows out of this initial example.

Do not add vision, realistic suction physics, cutting logic, or all four workstations until this small demo works.

## References

- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/
- Gazebo + ROS 2: https://gazebosim.org/docs/harmonic/ros2_overview/
- ROS 2 Python tutorials: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html
- GitHub pull requests: https://docs.github.com/en/pull-requests
