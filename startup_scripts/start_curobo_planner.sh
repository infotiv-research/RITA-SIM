#!/bin/bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
USER_WS="${USER_WS:-/ros2_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

log() {
  echo "[start_curobo_planner][$(timestamp)] $*"
}

log "Starting curobo launcher."
log "ROS_DISTRO=${ROS_DISTRO} USER_WS=${USER_WS}"

# Clear the curobo-specific stack so a fresh launch does not double-allocate GPU memory.
for pattern in \
  "ur_robotiq_curobo.launch.py" \
  "ur_robotiq_curobo_human.launch.py" \
  "curobo_trajectory_planner" \
  "curobo_human_skeleton_collision_publisher.py" \
  "curobo_live_collision_spheres.py" \
  "curobo_world_bridge.py" \
  "curobo_minimal.rviz" \
  "/opt/ros/.*/lib/rviz2/rviz2" \
  "/opt/ros/.*/lib/robot_state_publisher/robot_state_publisher" \
  "/opt/ros/.*/lib/joint_state_publisher/joint_state_publisher" \
  "/opt/ros/.*/lib/tf2_ros/static_transform_publisher .* preview/world"
do
  pkill -f "$pattern" 2>/dev/null || true
done
sleep 1

if [[ "${*:-}" != *"launch_rviz:=false"* ]]; then
  if [ -z "${DISPLAY:-}" ]; then
    log "WARNING: DISPLAY is not set. rviz2 will not be able to open a window."
  fi
  if [ -n "${XAUTHORITY:-}" ] && [ -d "${XAUTHORITY}" ]; then
    log "WARNING: XAUTHORITY points to a directory (${XAUTHORITY}), not an auth file."
    log "WARNING: This usually means the host-side Xauthority file did not exist when the container was created."
  elif [ -n "${XAUTHORITY:-}" ] && [ ! -e "${XAUTHORITY}" ]; then
    log "WARNING: XAUTHORITY does not exist at ${XAUTHORITY}."
  fi
fi

"${SCRIPT_DIR}/bootstrap_cumotion_workspace.sh"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${USER_WS}/install/setup.bash" ]; then
  source "${USER_WS}/install/setup.bash"
fi
set -u

if [[ "${CUROBO_BOOTSTRAP_HOME_IF_ZERO:-1}" != "0" ]]; then
  bootstrap_executable="${USER_WS}/install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/curobo_home_bootstrap.py"
  if [[ ! -f "${bootstrap_executable}" ]]; then
    bootstrap_executable="${USER_WS}/src/ur_robotiq_moveit_config/scripts/curobo/curobo_home_bootstrap.py"
  fi
  bootstrap_cmd=(
    python3
    "${bootstrap_executable}"
  )
  printf '[start_curobo_planner][%s] Bootstrap: ' "$(timestamp)"
  printf '%q ' "${bootstrap_cmd[@]}"
  echo
  "${bootstrap_cmd[@]}" || true
fi

launch_cmd=(
  ros2 launch ur_robotiq_curobo_config ur_robotiq_curobo_human.launch.py
  "$@"
)

printf '[start_curobo_planner][%s] Command: ' "$(timestamp)"
printf '%q ' "${launch_cmd[@]}"
echo

exec "${launch_cmd[@]}"
