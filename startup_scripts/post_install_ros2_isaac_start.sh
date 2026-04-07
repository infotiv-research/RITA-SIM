#!/bin/bash
set -euo pipefail
 
# --- Paths ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/isaac-sim}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
USER_WS="${USER_WS:-/ros2_ws}"
DEFAULT_USD_SCENE="${DEFAULT_USD_SCENE:-assets/ur10e_robotiq2f-140/main_scene.usd}"
ISAAC_STARTUP_OPEN_SCRIPT="${ISAAC_STARTUP_OPEN_SCRIPT:-${SCRIPT_DIR}/isaac_open_stage_startup.py}"
BRIDGE_LIB_PATH="${ISAAC_SIM_PATH}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"
 
# --- Sanity checks ---
[ -f "$ISAAC_SIM_PATH/isaac-sim.sh" ] || { echo "Error: Isaac Sim not found at $ISAAC_SIM_PATH"; exit 1; }
[ -d "$BRIDGE_LIB_PATH" ] || { echo "Error: Isaac Sim ROS 2 bridge libs not found at $BRIDGE_LIB_PATH"; exit 1; }
 
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
 
# --- Core env ---
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export FASTDDS_SHM_DEFAULT_SEGMENT_SIZE="${FASTDDS_SHM_DEFAULT_SEGMENT_SIZE:-134217728}"  # 128MB
 
# --- Debug ---
echo "ROS_DISTRO=$ROS_DISTRO"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Isaac launch will use bundled ${ROS_DISTRO} bridge paths only"

launch_isaac() {
  exec env \
    -u PYTHONPATH \
    -u OLD_PYTHONPATH \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    LD_LIBRARY_PATH="$BRIDGE_LIB_PATH" \
    "$ISAAC_SIM_PATH/isaac-sim.sh" --allow-root "$@"
}
 
# --- Launch ---
if [ -n "$SCENE_PATH" ]; then
  echo "Opening scene via startup hook: $SCENE_PATH"
  export ISAAC_STARTUP_SCENE="$SCENE_PATH"
  export ISAAC_STARTUP_SCENE_WAIT_UPDATES="${ISAAC_STARTUP_SCENE_WAIT_UPDATES:-5}"
  launch_isaac --exec "$ISAAC_STARTUP_OPEN_SCRIPT" "$@"
else
  launch_isaac "$@"
fi
