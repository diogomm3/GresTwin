# GresTwin Pick-and-Place

Development environment for a future ROS 2 robot-arm pick-and-place simulation. The repository currently contains only workspace/tooling scaffolding; no robot, world, or simulation has been implemented.

## Recommended stack

This machine is Ubuntu 24.04.4 LTS (Noble) under WSL2, so use the matching stable stack:

- **ROS 2 Jazzy Jalisco** — the ROS 2 LTS release for Ubuntu 24.04.
- **Gazebo Harmonic** — the recommended Gazebo LTS paired with Jazzy.
- **MoveIt 2 Jazzy** — install the binary release rather than building it from source.
- **ros2_control** and **gz_ros2_control** — ROS controller manager and its Gazebo system plugin.
- **ros_gz** — ROS/Gazebo transport and simulation integration.
- **Python 3.12 + uv** — uv manages project-local developer tools; ROS Python packages remain those installed by Ubuntu/ROS.
- **rosdep + colcon** — install ROS package dependencies and build packages in `ros_ws`.

Use ROS Jazzy binary packages for the selected components, with Gazebo Harmonic supplied through the matching ROS/Gazebo integration. Do not add a separate OSRF Gazebo apt repository or mix ROS/Gazebo release versions.

## Install ROS system packages

On Ubuntu 24.04, follow the [official ROS 2 Jazzy Ubuntu binary-package instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), including configuring the ROS apt repository. Ensure Ubuntu's `universe`, `noble-updates`, and `noble-backports` apt suites are enabled, as well as a UTF-8 locale. After configuring the ROS repository, run `sudo apt update` before installing packages. The [ROS/Gazebo compatibility guidance](https://gazebosim.org/docs/harmonic/ros_installation/) recommends Jazzy with Harmonic; do not add a separate OSRF Gazebo apt repository.

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

Initialize rosdep once per machine, then update its package index:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` reports that rosdep is already initialized, continue with `rosdep update`.

Source ROS in each shell before using ROS commands:

```bash
source /opt/ros/jazzy/setup.bash
```

## Set up the project

Create the local Python tooling environment with uv (this does not install or replace system Python or ROS packages):

```bash
uv sync
uv run ruff check .
```

ROS packages will live under `ros_ws/src/`. Once packages are added, install their declared system dependencies and build from the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ros_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

`ros_ws/build`, `ros_ws/install`, and `ros_ws/log` are generated artifacts and are ignored by Git.

## WSL2 note

WSLg and the WSL GPU device are present on this machine, so Gazebo and RViz GUI applications may work, subject to the Windows graphics driver. Verify rendering after installation; headless simulation remains an option if rendering is unreliable. WSL2 is suitable for development and simulation, not hard real-time control or direct hardware timing.
