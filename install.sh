#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -r /etc/os-release ]]; then
    printf 'Cannot determine the operating system from /etc/os-release.\n' >&2
    exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != "24.04" ]]; then
    printf 'This installer supports Ubuntu 24.04 only; detected %s %s.\n' \
        "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
    exit 1
fi

sudo_cmd=()
if (( EUID == 0 )); then
    printf 'Run this installer as your regular user; it invokes sudo when needed.\n' >&2
    exit 1
fi
if ! command -v sudo >/dev/null 2>&1; then
    printf 'sudo is required to install system packages.\n' >&2
    exit 1
fi
sudo_cmd=(sudo)

"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    locales \
    software-properties-common

"${sudo_cmd[@]}" add-apt-repository -y universe
"${sudo_cmd[@]}" locale-gen en_US.UTF-8
"${sudo_cmd[@]}" update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null \
    | grep -q 'install ok installed'; then
    ros_apt_source_version="$(
        curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
            | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])'
    )"
    ros_apt_source_deb="ros2-apt-source_${ros_apt_source_version}.noble_all.deb"
    ros_apt_source_path="$(mktemp --suffix=.deb)"
    trap 'rm -f "$ros_apt_source_path"' EXIT
    curl -fL --retry 3 \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/${ros_apt_source_deb}" \
        -o "$ros_apt_source_path"
    "${sudo_cmd[@]}" dpkg -i "$ros_apt_source_path"
    rm -f "$ros_apt_source_path"
    trap - EXIT
fi

"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    python3-rosdep \
    ros-dev-tools

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    "${sudo_cmd[@]}" rosdep init
fi
rosdep update

curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="${HOME}/.local/bin:${PATH}"
uv sync --project "$repo_root"

source /opt/ros/jazzy/setup.bash
cd "$repo_root/ros_ws"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install

printf '\nInstallation and workspace build complete.\n'
printf 'For each new shell, run:\n'
printf '  source /opt/ros/jazzy/setup.bash\n'
printf '  source "%s/ros_ws/install/setup.bash"\n' "$repo_root"
