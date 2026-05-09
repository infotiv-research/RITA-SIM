#!/bin/bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
USER_WS="${USER_WS:-/ros2_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

log() {
  echo "[start_ompl_planner][$(timestamp)] $*"
}

log "Starting OMPL planner launcher."
log "ROS_DISTRO=${ROS_DISTRO} USER_WS=${USER_WS}"

set +u
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  log "Sourced /opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [ -f /opt/vendor_ws/install/local_setup.bash ]; then
  source /opt/vendor_ws/install/local_setup.bash
  log "Sourced /opt/vendor_ws/install/local_setup.bash"
fi
if [ -f "${USER_WS}/install/local_setup.bash" ]; then
  source "${USER_WS}/install/local_setup.bash"
  log "Sourced ${USER_WS}/install/local_setup.bash"
fi
set -u

if [[ "${OMPL_BOOTSTRAP_HOME_IF_ZERO:-1}" != "0" ]]; then
  bootstrap_executable="${USER_WS}/install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/home_bootstrap.py"
  if [[ ! -f "${bootstrap_executable}" ]]; then
    bootstrap_executable="${USER_WS}/src/ur_robotiq_moveit_config/scripts/home_bootstrap.py"
  fi
  bootstrap_cmd=(
    python3
    "${bootstrap_executable}"
  )
  printf '[start_ompl_planner][%s] Bootstrap: ' "$(timestamp)"
  printf '%q ' "${bootstrap_cmd[@]}"
  echo
  "${bootstrap_cmd[@]}" || true
fi

launch_cmd=(
  ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py
  planning_pipeline:=ompl
  launch_cumotion_planner:=false
  publish_live_collision_spheres:=true
  "$@"
)

printf '[start_ompl_planner][%s] Command: ' "$(timestamp)"
printf '%q ' "${launch_cmd[@]}"
echo

exec "${launch_cmd[@]}"
