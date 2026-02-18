#!/bin/bash
set -e

cd /ros2_ws

echo "Running colcon build on container start..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
echo "colcon build complete."
