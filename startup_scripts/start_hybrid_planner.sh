#!/bin/bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
USER_WS="${USER_WS:-/ros2_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

log() {
  echo "[start_hybrid_planner][$(timestamp)] $*"
}

log "Starting cuRobo hybrid planner launcher."
log "ROS_DISTRO=${ROS_DISTRO} USER_WS=${USER_WS}"

if [ "${HYBRID_SKIP_BOOTSTRAP:-false}" != "true" ]; then
  log "Running bootstrap/validation checks before launch."
  "${SCRIPT_DIR}/bootstrap_cumotion_workspace.sh"
else
  log "Skipping bootstrap because HYBRID_SKIP_BOOTSTRAP=true."
fi

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

launch_cmd=(
  ros2 launch curobo_hybrid_planning_plugins curobo_hybrid_planning.launch.py
  "$@"
)

log "Launching MoveIt + Hybrid Planning."
printf '[start_hybrid_planner][%s] Command: ' "$(timestamp)"
printf '%q ' "${launch_cmd[@]}"
echo

exec "${launch_cmd[@]}"
