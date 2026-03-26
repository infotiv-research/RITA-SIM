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

detect_gpu_vram_mb() {
  local line total free
  if command -v nvidia-smi >/dev/null 2>&1; then
    line="$(nvidia-smi --query-gpu=memory.total,memory.free --format=csv,noheader,nounits 2>/dev/null | head -n 1 || true)"
    if [ -n "${line}" ]; then
      total="$(echo "${line}" | cut -d',' -f1 | tr -dc '0-9')"
      free="$(echo "${line}" | cut -d',' -f2 | tr -dc '0-9')"
      if [ -n "${total}" ]; then
        echo "${total}:${free:-0}"
        return 0
      fi
    fi
  fi

  return 1
}

select_curobo_profile() {
  local total_mb="$1"
  if [ "${total_mb}" -le 8192 ]; then
    echo "low"
  else
    echo "high"
  fi
}

log "Starting curobo launcher."
log "ROS_DISTRO=${ROS_DISTRO} USER_WS=${USER_WS}"

export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
log "PYTORCH_CUDA_ALLOC_CONF=${PYTORCH_CUDA_ALLOC_CONF}"

detected_total_vram_mb=""
detected_free_vram_mb=""
selected_profile="${CUROBO_PROFILE:-auto}"
if vram_pair="$(detect_gpu_vram_mb)"; then
  detected_total_vram_mb="${vram_pair%%:*}"
  detected_free_vram_mb="${vram_pair##*:}"
fi

if [ "${selected_profile}" = "auto" ]; then
  if [ -n "${detected_total_vram_mb}" ]; then
    selected_profile="$(select_curobo_profile "${detected_total_vram_mb}")"
  else
    selected_profile="high"
  fi
fi

if [ -n "${detected_total_vram_mb}" ] && [ -n "${detected_free_vram_mb}" ] && [ "${detected_total_vram_mb}" -gt 0 ] && [ "${detected_free_vram_mb}" -gt 0 ]; then
  free_pct=$((100 * detected_free_vram_mb / detected_total_vram_mb))
  if [ "${free_pct}" -lt 20 ] && [ "${selected_profile}" = "high" ]; then
    selected_profile="low"
  fi
fi

if [ -n "${detected_total_vram_mb}" ]; then
  log "Detected GPU0 VRAM total=${detected_total_vram_mb}MiB free=${detected_free_vram_mb}MiB -> curobo profile=${selected_profile}"
else
  log "Could not detect GPU VRAM -> using curobo profile=${selected_profile}"
fi

selected_planner_params_file="${USER_WS}/src/ur_robotiq_curobo_config/config/curobo_planner_params.yaml"
if [ "${selected_profile}" = "low" ]; then
  selected_planner_params_file="${USER_WS}/src/ur_robotiq_curobo_config/config/curobo_planner_params_8gb.yaml"
fi

log "curobo planner params file=${selected_planner_params_file}"

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
if [ -f "${USER_WS}/install/local_setup.bash" ]; then
  source "${USER_WS}/install/local_setup.bash"
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
  planner_params_file:=${selected_planner_params_file}
  "$@"
)

printf '[start_curobo_planner][%s] Command: ' "$(timestamp)"
printf '%q ' "${launch_cmd[@]}"
echo

exec "${launch_cmd[@]}"
