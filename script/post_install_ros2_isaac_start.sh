#!/bin/bash
set -euo pipefail

# --- Paths ---
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/isaac-sim}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
USER_WS="${USER_WS:-/ros2_ws}"

# --- Sanity checks ---
[ -f "$ISAAC_SIM_PATH/isaac-sim.sh" ] || { echo "Error: Isaac Sim not found at $ISAAC_SIM_PATH"; exit 1; }

# --- Source ROS 2 (system + workspace) with nounset relaxed ---
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  # silence ament setup chatter & guard python var for some setups
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$(command -v python3 || echo /usr/bin/python3)}"

  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  [ -f "${USER_WS}/install/setup.bash" ] && source "${USER_WS}/install/setup.bash"
  set -u
else
  echo "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found"
fi

# --- Core env ---
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export FASTDDS_SHM_DEFAULT_SEGMENT_SIZE="${FASTDDS_SHM_DEFAULT_SEGMENT_SIZE:-134217728}"  # 128MB

# --- Isaac ROS bridge libs ---
export isaac_sim_package_path="$ISAAC_SIM_PATH"
BRIDGE_LIB_PATH="$isaac_sim_package_path/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"
case ":${LD_LIBRARY_PATH:-}:" in *":$BRIDGE_LIB_PATH:"*) ;; *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$BRIDGE_LIB_PATH";; esac

# --- Debug (helpful when importer fails) ---
echo "ROS_DISTRO=$ROS_DISTRO"
echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-<unset>}"
echo "COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-<unset>}"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
which ros2 || true
ros2 pkg prefix robot_state_publisher || echo "robot_state_publisher not found"

# --- Launch ---
exec "$ISAAC_SIM_PATH/isaac-sim.sh" --allow-root
