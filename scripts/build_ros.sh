#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
project_root="$PWD"
# Colcon environment hooks need a build prefix without spaces.
ros_build_root="${FWRL_ROS_BUILD_ROOT:-$HOME/.local/share/fwrl/ros2_ws}"
mkdir -p "$ros_build_root"
cd "$ros_build_root"
/usr/bin/python3 -m colcon build --base-paths "$project_root/ros2_ws/src" --symlink-install
