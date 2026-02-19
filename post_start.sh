#!/bin/bash
set -e

cd /ros2_ws

if [ ! -d "build" ] || [ -z "$(ls -A build)" ]; then
  echo "🚧 No build/ folder found. Running colcon build..."
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --symlink-install
  echo "✅ colcon build complete."
else
  echo "✅ build/ folder already exists. Skipping build."
fi
