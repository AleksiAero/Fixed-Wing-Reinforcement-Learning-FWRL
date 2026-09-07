#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43
export PYTHONPATH="$PWD:$PWD/ros2_ws/src/fwrl_ros${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 scripts/ros_smoke.py
