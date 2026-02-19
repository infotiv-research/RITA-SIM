#!/bin/bash
set -euo pipefail
 
# --- Paths ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/isaac-sim}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
USER_WS="${USER_WS:-/ros2_ws}"
DEFAULT_USD_SCENE="${DEFAULT_USD_SCENE:-assets/ur10e_robotiq2f-140/scene_with_flowrack_and_crates2.usd}"
ISAAC_STARTUP_OPEN_SCRIPT="${ISAAC_STARTUP_OPEN_SCRIPT:-${SCRIPT_DIR}/isaac_open_stage_startup.py}"
 
# --- Sanity checks ---
[ -f "$ISAAC_SIM_PATH/isaac-sim.sh" ] || { echo "Error: Isaac Sim not found at $ISAAC_SIM_PATH"; exit 1; }
 
SCENE_PATH="$DEFAULT_USD_SCENE"
if [ "$#" -gt 0 ] && [[ "$1" != -* ]]; then
  SCENE_PATH="$1"
  shift
fi
 
# Resolve relative scene paths from the workspace root.
if [ -n "$SCENE_PATH" ] && [[ "$SCENE_PATH" != /* ]]; then
  SCENE_PATH="${USER_WS}/${SCENE_PATH}"
fi
 
if [ -n "$SCENE_PATH" ] && [ ! -f "$SCENE_PATH" ]; then
  echo "Warning: scene file not found: $SCENE_PATH"
  echo "Starting Isaac Sim without preloaded scene."
  SCENE_PATH=""
fi
 
# Skip startup preload if caller already provides a custom --exec script.
if [ -n "$SCENE_PATH" ]; then
  for arg in "$@"; do
    if [ "$arg" = "--exec" ] || [ "$arg" = "-e" ]; then
      echo "Warning: --exec argument detected, skipping automatic scene preload."
      SCENE_PATH=""
      break
    fi
  done
fi
 
if [ -n "$SCENE_PATH" ] && [ ! -f "$ISAAC_STARTUP_OPEN_SCRIPT" ]; then
  echo "Warning: startup open script not found: $ISAAC_STARTUP_OPEN_SCRIPT"
  echo "Starting Isaac Sim without preloaded scene."
  SCENE_PATH=""
fi
 
# --- Source ROS 2 (system + workspace) ---
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  # silence ament setup chatter & guard python var for some setups
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
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
if [ -n "$SCENE_PATH" ]; then
  echo "Opening scene via startup hook: $SCENE_PATH"
  export ISAAC_STARTUP_SCENE="$SCENE_PATH"
  export ISAAC_STARTUP_SCENE_WAIT_UPDATES="${ISAAC_STARTUP_SCENE_WAIT_UPDATES:-5}"
  exec "$ISAAC_SIM_PATH/isaac-sim.sh" --allow-root --exec "$ISAAC_STARTUP_OPEN_SCRIPT" "$@"
else
  exec "$ISAAC_SIM_PATH/isaac-sim.sh" --allow-root "$@"
fi