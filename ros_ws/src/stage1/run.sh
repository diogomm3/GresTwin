#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_ws_dir="$(cd -- "$script_dir/../.." && pwd)"
ros_setup="/opt/ros/jazzy/setup.bash"

if [[ ! -f "$ros_setup" ]]; then
    printf 'ROS 2 Jazzy setup not found: %s\n' "$ros_setup" >&2
    exit 1
fi

source "$ros_setup"
cd "$ros_ws_dir"
colcon build --symlink-install --packages-up-to stage1_stoneware_sim
source install/setup.bash
exec ros2 launch stage1_stoneware_sim stage1.launch.py gazebo_gui:=true
