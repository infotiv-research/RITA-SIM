#!/bin/bash
set -e

cd /ros2_ws

echo "🔧 Running rosdep update..."
rosdep update

echo "📦 Installing dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y --skip-keys=keyboard
if [ ! -d "src/serial/.git" ]; then
    git clone -b ros2 https://github.com/tylerjw/serial.git src/serial
else
    echo "Serial package already exists, skipping clone"
fi
echo "✅ Done!"
