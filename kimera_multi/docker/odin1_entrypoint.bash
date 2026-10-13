#!/usr/bin/env bash
set -e
mkdir -p "$HOME/.ros"
source /opt/ros/noetic/setup.bash
source /opt/kimera_ws/devel/setup.bash
# Prefer the host's current launch/parameter files without rebuilding C++.
if [[ -f /workspace/package.xml ]]; then
  export ROS_PACKAGE_PATH="/workspace:${ROS_PACKAGE_PATH}"
fi
exec "$@"
