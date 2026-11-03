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
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    ROS_DOMAIN_ID="$(python3 -c 'import secrets; print(secrets.randbelow(101) + 1)')"
    export ROS_DOMAIN_ID
fi
printf 'Using ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
if [[ -z "${GZ_PARTITION:-}" ]]; then
    GZ_PARTITION="stage1_${ROS_DOMAIN_ID}"
    export GZ_PARTITION
fi
printf 'Using GZ_PARTITION=%s\n' "$GZ_PARTITION"
cd "$ros_ws_dir"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-up-to stage1_stoneware_sim
source install/setup.bash
exec ros2 launch stage1_stoneware_sim stage1.launch.py "$@"
