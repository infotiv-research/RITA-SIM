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

select_hybrid_profile() {
  local total_mb="$1"
  if [ "${total_mb}" -le 8192 ]; then
    echo "8gb"
  else
    echo "default"
  fi
}

log "Starting cuRobo hybrid planner launcher."
log "ROS_DISTRO=${ROS_DISTRO} USER_WS=${USER_WS}"

export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True,max_split_size_mb:64}"
log "PYTORCH_CUDA_ALLOC_CONF=${PYTORCH_CUDA_ALLOC_CONF}"

detected_total_vram_mb=""
detected_free_vram_mb=""
selected_profile="${HYBRID_PROFILE:-auto}"
if vram_pair="$(detect_gpu_vram_mb)"; then
  detected_total_vram_mb="${vram_pair%%:*}"
  detected_free_vram_mb="${vram_pair##*:}"
fi

if [ "${selected_profile}" = "auto" ]; then
  if [ -n "${detected_total_vram_mb}" ]; then
    selected_profile="$(select_hybrid_profile "${detected_total_vram_mb}")"
  else
    selected_profile="default"
  fi
fi

selected_planner_params_file="${USER_WS}/src/ur_robotiq_curobo_config/config/curobo_hybrid_planner_params.yaml"
if [ "${selected_profile}" = "8gb" ]; then
  selected_planner_params_file="${USER_WS}/src/ur_robotiq_curobo_config/config/curobo_hybrid_planner_params_8gb.yaml"
fi

if [ -n "${HYBRID_PLANNER_PARAMS_FILE:-}" ]; then
  selected_planner_params_file="${HYBRID_PLANNER_PARAMS_FILE}"
fi

if [ -n "${detected_total_vram_mb}" ]; then
  log "Detected GPU0 VRAM total=${detected_total_vram_mb}MiB free=${detected_free_vram_mb}MiB -> hybrid profile=${selected_profile}"
else
  log "Could not detect GPU VRAM -> using hybrid profile=${selected_profile}"
fi
log "hybrid planner params file=${selected_planner_params_file}"

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

if [[ "${HYBRID_BOOTSTRAP_HOME_IF_ZERO:-1}" != "0" ]]; then
  bootstrap_executable="${USER_WS}/install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/home_bootstrap.py"
  if [[ ! -f "${bootstrap_executable}" ]]; then
    bootstrap_executable="${USER_WS}/src/ur_robotiq_moveit_config/scripts/home_bootstrap.py"
  fi
  bootstrap_cmd=(
    python3
    "${bootstrap_executable}"
  )
  printf '[start_hybrid_planner][%s] Bootstrap: ' "$(timestamp)"
  printf '%q ' "${bootstrap_cmd[@]}"
  echo
  "${bootstrap_cmd[@]}" || true
fi

launch_cmd=(
  ros2 launch curobo_hybrid_planning_plugins curobo_hybrid_planning.launch.py
  planner_params_file:=${selected_planner_params_file}
  "$@"
)

log "Launching MoveIt + Hybrid Planning."
printf '[start_hybrid_planner][%s] Command: ' "$(timestamp)"
printf '%q ' "${launch_cmd[@]}"
echo

exec "${launch_cmd[@]}"
