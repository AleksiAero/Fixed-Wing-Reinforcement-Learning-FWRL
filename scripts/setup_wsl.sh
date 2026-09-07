#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo 'Ubuntu 24.04 required'; exit 1; }
if [[ $EUID -eq 0 ]]; then SUDO=(); else SUDO=(sudo); fi
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y curl ca-certificates locales python3-venv python3-pip python3-tk git blender
"${SUDO[@]}" locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
if [[ ! -f /etc/apt/sources.list.d/ros2.sources ]]; then
  version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  curl -fsSL -o /tmp/fwrl-ros-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.noble_all.deb"
  "${SUDO[@]}" dpkg -i /tmp/fwrl-ros-source.deb
fi
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep
bash scripts/setup_python.sh
echo 'WSL dependencies ready. Run: bash scripts/build_ros.sh'
