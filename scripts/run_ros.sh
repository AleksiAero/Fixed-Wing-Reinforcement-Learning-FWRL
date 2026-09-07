#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source "${FWRL_ROS_BUILD_ROOT:-$HOME/.local/share/fwrl/ros2_ws}/install/setup.bash"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
exec ros2 launch fwrl_ros simulation.launch.py world:="$(pwd)/worlds/training_airfield.json" "$@"
